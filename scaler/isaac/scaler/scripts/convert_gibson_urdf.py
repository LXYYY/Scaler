import argparse
import asyncio
import os

import omni
from omni.isaac.kit import SimulationApp


async def convert(in_file, out_file, load_materials=False):
    # This import causes conflicts when global
    import omni.kit.asset_converter

    def progress_callback(progress, total_steps):
        pass

    converter_context = omni.kit.asset_converter.AssetConverterContext()
    # setup converter and flags
    converter_context.ignore_materials = not load_materials
    # converter_context.ignore_animation = False
    # converter_context.ignore_cameras = True
    # converter_context.single_mesh = True
    # converter_context.smooth_normals = True
    # converter_context.preview_surface = False
    # converter_context.support_point_instancer = False
    # converter_context.embed_mdl_in_usd = False
    # converter_context.use_meter_as_world_unit = True
    # converter_context.create_world_as_default_root_prim = False
    instance = omni.kit.asset_converter.get_instance()
    task = instance.create_converter_task(in_file, out_file, progress_callback, converter_context)
    success = True
    while True:
        success = await task.wait_until_finished()
        if not success:
            await asyncio.sleep(0.1)
        else:
            break
    return success


def asset_convert(args):
    supported_file_formats = ["stl", "obj", "fbx"]
    for folder in args.folders:
        local_asset_output = folder + "_converted"
        result = omni.client.create_folder(f"{local_asset_output}")

    for folder in args.folders:
        print(f"\nConverting folder {folder}...")

        (result, models) = omni.client.list(folder)
        for i, entry in enumerate(models):
            if i >= args.max_models:
                print(f"max models ({args.max_models}) reached, exiting conversion")
                break

            model = str(entry.relative_path)
            model_name = os.path.splitext(model)[0]
            model_format = (os.path.splitext(model)[1])[1:]
            # Supported input file formats
            if model_name == "mesh" and model_format in supported_file_formats:
                input_model_path = folder + "/" + model
                converted_model_path = folder + "_converted/" + model_name + "_" + model_format + ".usd"
                if not os.path.exists(converted_model_path):
                    status = asyncio.get_event_loop().run_until_complete(
                        convert(input_model_path, converted_model_path, True)
                    )
                    if not status:
                        print(f"ERROR Status is {status}")
                    print(f"---Added {converted_model_path}")


def scan_folder(path: str):
    """Scan subfolders for obj files

    Args:
        path (str): Path to the folder to scan.

    Returns:
        list: List of obj files and their paths.
    """

    # scan all subfolders for mesh.obj files
    model_paths = []
    for root, dirs, files in os.walk(path):
        for file in files:
            if file == "mesh.obj":
                model_paths.append(os.path.join(root))

    return model_paths


if __name__ == "__main__":
    kit = SimulationApp()

    from omni.isaac.core.utils.extensions import enable_extension

    enable_extension("omni.kit.asset_converter")

    # parse args for directories to convert
    import argparse

    parser = argparse.ArgumentParser(description="Converts all mesh.obj files in a folder to usd")
    parser.add_argument("--path", type=str, help="Path to the folder to convert")
    parser.add_argument("--folders", type=str, nargs="+", help="Folders to convert")
    parser.add_argument("--max_models", type=int, default=100000, help="Maximum number of models to convert")
    args = parser.parse_args()

    scan_folder(args.path)

    # print all obj files and their paths
    for path in scan_folder(args.path):
        print(path)

    args.folders = scan_folder(args.path)
    asset_convert(args)

    kit.close()
