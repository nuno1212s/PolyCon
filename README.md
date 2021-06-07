# PolyCon

PolyCon is a program made for generating random N vertices simple polygons within an MxM space.

## Compilation


```bash
mkdir build && cd build
cmake ..
make
```

## Usage

From the folder where the PolyCon executable is found

```bash
./PolyCon N M
```

## Visualizer

To visualize a generated polygon you can use the included visualizer.

```bash
python3 visualizer.py M result.json
```

Note that if the visualizer.py is not in the same folder as the result.json, you have to indicate the path to the result.json file.

## DCEL

The DCEL implemented in this program is specialized for generating polygons by repeated triangulation, so it will probably not work as a generalized DCEl.
See the report that is present in this repository to see the proof of correctness for the implemented DCEL in the use case of this assignment

## License
[MIT](https://choosealicense.com/licenses/mit/)