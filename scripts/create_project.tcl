set proj_name dragon_ai
set proj_dir ./build/vivado

create_project $proj_name $proj_dir -part xczu7ev-ffvc1156-2-e

# RTL
add_files [glob ./hw/**/*.sv]

# Top
set_property top top [current_fileset]

update_compile_order -fileset sources_1
