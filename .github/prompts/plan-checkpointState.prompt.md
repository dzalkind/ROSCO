# Plan: Controller State Checkpointing

## TL;DR
Every filter, PI/PID controller and rate limiter whose state persists between
timesteps lives in one struct, `ControllerObjects` (`src/include/controller_objects.hpp`),
with a single instance `ObjState`. `WriteRestartFile`/`ReadRestartFile` serialise
it as one raw block, which is what makes an OpenFAST warm restart
(`iStatus == -9`) bit-exact. Adding new persistent state means adding a member
to that struct — see *Adding new controller state* below.

## Status — COMPLETE
- 64 objects across 17 modules moved out of function-local `static`s into `ObjState`.
- `ObjState` serialised in `checkpoint_fields()` (`src/include/restart_fields.h`).
- `iStatus -8`/`-9` no longer run stages 3-7, so checkpointing does not perturb
  the run it captures and a restart does not resume one step ahead.
- `n_DT` and the ZeroMQ poll skipped on `-8`/`-9` for the same reason.
- Verified: `rosco/test/test_checkpoint.py` passes with **zero** difference in
  `GenPwr`, `BldPitch1`, `GenTq` and `RotSpeed` between the restarted and the
  uninterrupted run; 27/27 regression scenarios byte-identical.

## The constraint
Checkpointing requires the controller's state to be **enumerable**. There is no
design in which state can be declared anywhere with no bookkeeping *and* be
saved and restored. The only open question is whether the enumeration is a
central list maintained by hand, or something the declaration performs for
itself. We currently use the central list.

## Adding new controller state
1. Add the member to the owning module's nested struct in `controller_objects.hpp`.
2. Use it as `ObjState.<module>.<name>` in the module.
3. Do **not** declare a new function-local `static` filter/controller/limiter.
   It will work for a normal run and silently corrupt a warm restart.

The classes must stay flat bags of arithmetic members — a `static_assert` on
`std::is_trivially_copyable` in the header enforces this, because the checkpoint
writes `ControllerObjects` as raw bytes. Adding a pointer, `std::vector` or
`std::string` to any filter or controller class will trip it.

Note that the checkpoint file is only readable by the binary that wrote it:
struct layout is the format. That is acceptable because OpenFAST restarts use
the same library that wrote the checkpoint.

## Known weakness
Step 3 above is a rule a maintainer has to remember. Forgetting it produces no
compiler error and no test failure unless someone exercises restart on that
specific code path. The alternatives below trade that hazard for other costs.

## Alternatives for later

### A. Self-registering state (`Persist<T>`)
Make declaring the state *be* the registration:

```cpp
struct StateBlock { void* ptr; std::size_t size; };
std::vector<StateBlock>& state_blocks();

template <class T>
struct Persist : T {
    Persist() { state_blocks().push_back({static_cast<T*>(this), sizeof(T)}); }
};
```

Declare `Persist<LPFilter> genSpeedFilter;` at namespace scope in the module
that owns it; every call site stays `genSpeedFilter.init(...)` / `.step(...)`
unchanged. `controller_objects.hpp` and the `ObjState.<module>.` prefixes
disappear, and the call sites get shorter than they are today.

- **Gains**: state is declared once, in the module that owns it, and cannot be
  forgotten. No central file to keep in sync.
- **Costs**: objects must be at namespace scope so they all construct at library
  load, before any restore can run. A `Persist` declared inside a function would
  register lazily and corrupt the ordering — guard by snapshotting
  `state_blocks().size()` at `iStatus == 0` and throwing if it ever grows.
  Serialisation becomes registration-order rather than struct-layout order,
  which is the same portability story as today.
- `int Tidx` in `yawratecontrol.cpp` cannot inherit from `int`, so it becomes a
  one-field struct. One extra line, no second template.

This is the option to reach for if the central struct starts drifting.

### B. Keep the struct, enforce it in CI
Add a test that greps the controller sources for `static <StatefulType>` and
fails if any appear outside `controller_objects.cpp`. Roughly fifteen lines,
zero churn, zero runtime cost. Converts the silent failure in *Known weakness*
into a loud one, but does not restore the flexibility of declaring state locally.

Cheapest way to de-risk the current design without changing it.

### C. Thread a state struct through the call chain
Pass the state explicitly as a parameter, the way the Fortran did with
`LocalVar%FP`. Most explicit of the three and needs no static storage at all,
but it touches every function signature in the controller — more churn than A
or B for the same result. Not recommended unless the controller is being
restructured for another reason anyway.

### D. Named blocks instead of ordered blocks
Serialise `key -> bytes` rather than a positional array, with the key generated
from `__FILE__`/`__LINE__`. Removes the static-initialisation-order constraint
in A entirely, since a lazily constructed object can pull its own bytes out of a
pending-restore map the moment it comes into existence. Costs a map, a key
scheme, and checkpoints that break whenever line numbers move. Only worth it if
lazily constructed state becomes unavoidable.

## Related gotchas found while fixing this
- `accINFILE` only holds the controller input file on the first call. OpenFAST
  reuses that argument to pass `<RootName>.dll.chkp` on `iStatus -8`/`-9`, so
  `LocalVar.ACC_INFILE` is latched at `iStatus == 0` and never overwritten.
  It is checkpointed and replayed into `read_config_files()` on restore.
- `LocalVar.FP`, `piP`, `resP` and `rlP` are dead — the per-instance vectors
  from the original translation, still serialised but no longer read by anything.
  They can be dropped from `checkpoint_fields()` whenever someone is willing to
  invalidate existing checkpoint files.
