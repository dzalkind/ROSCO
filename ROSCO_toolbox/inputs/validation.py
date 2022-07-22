import os
from wisdem.inputs import validate_with_defaults, simple_types, write_yaml

schema_dir = os.path.dirname(os.path.abspath(__file__))

def load_rosco_yaml(finput):
    rosco_schema = os.path.join(schema_dir,'toolbox_schema.yaml')
    return validate_with_defaults(finput, rosco_schema)

def write_rosco_yaml(instance, foutput):
    # Skip validation
    # weis_schema = get_modeling_schema()
    # validate_without_defaults(instance, weis_schema)
    sfx_str = ".yaml"
    if foutput[-5:] == sfx_str:
        foutput = foutput[-5:]
    elif foutput[-4:] == ".yml":
        foutput = foutput[-4:]
    sfx_str = "-rosco.yaml"
    instance2 = simple_types(instance)
    write_yaml(instance2, foutput+sfx_str)


if __name__=='__main__':
    fname = '/Users/dzalkind/Tools/ROSCO/Tune_Cases/NREL5MW.yaml'
    new_input = load_rosco_yaml(fname)
    
    print('here')

