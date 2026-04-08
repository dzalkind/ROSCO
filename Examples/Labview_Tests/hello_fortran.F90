subroutine hello_fortran(input_string) bind(C, name="hello_fortran")
    use, intrinsic :: iso_c_binding, only : c_char, c_null_char
    implicit none

    ! Match the fixed-size character buffer allocated in the C caller.
    integer, parameter :: buffer_len = 100
    ! Accept a null-terminated C string buffer and modify it in place.
    character(kind=c_char), intent(inout) :: input_string(*)
    character(len=50) :: suffix
    integer :: copy_len
    integer :: input_len
    integer :: index

    ! Write a simple status line to fort.10.
    suffix = "_fortran"
    open(unit=10, file="fort.10", status="replace", action="write")
    write(10, '(A)') "Hello from hello_fortran"
    close(unit=10)

    ! Find the current string length by scanning for the null terminator.
    input_len = 0
    do index = 1, buffer_len
        if (input_string(index) == c_null_char) exit
        input_len = index
    end do

    ! Append as much of the suffix as fits, leaving room for a terminator.
    copy_len = min(len(suffix), buffer_len - input_len - 1)
    do index = 1, copy_len
        input_string(input_len + index) = suffix(index:index)
    end do

    ! Re-terminate the C string after the appended suffix.
    if (input_len + copy_len + 1 <= buffer_len) then
        input_string(input_len + copy_len + 1) = c_null_char
    end if
end subroutine hello_fortran
