# MeshCartographyLib

**MeshCartographyLib** is a comprehensive geometry processing library designed to seamlessly bridge the realms of 2D and 3D spaces. Rooted in the principles of cartography, it offers tools to transform 3D meshes into 2D representations, akin to creating maps. Whether you're aiming to generate Escher-like tessellations or free-border 2D meshes from closed 3D structures, or harness the power of particle interactions on these meshes, this library facilitates it all.

## Compile

To compile the library, simply clone the repository and run the following command:

```bash
make
```

## Usage

```bash
./build/MeshCartography
```

or for testing:

```bash
./build/MeshCartography_test
```

And for all the python enthusiasts:

```bash
python3.13 -m venv venv
python3.13 mesh_cartographer.py
python3.13 generate_report.py
```

## Develop

```bash
find . -type f \( -name "*.cpp" -o -name "*.h" \) ! -name "argparse.hpp" -exec clang-format -i {} \;
```

### vcpkg manager

Work with vcpkg using this command structure logic:

```bash
./vcpkg/vcpkg list
./vcpkg/vcpkg install <package>
```
