# MtGRansac: Multi-threaded generic RANSAC implemetation

This is a header-only, multi-threaded implementation of the RANSAC algorithm, inspired by and based on [GRANSAC](https://github.com/drsrinathsridhar/GRANSAC).


## Dependencies

This library uses *C++11* features, so a suitable compiler is required (GCC 4.7+, 
Visual Studio 2013+). Additionally, *OpenMP* is needed for multi-threading.

## Usage

Just include the header RANSAC.hpp in your application. The Model class and InputData class
needs to be inherited to implement a suitable model for your application.

## Example: Plane and line fitting

To demonstrate how to use the library a plane and line fitting example is included.
To build this example do the following:

```bash
$ pwd
<SOME_DIR>/MtGRansac
$ mkdir build && cd build
$ cmake ../examples/
$ make
```

Running FittingSample should output files:
"plane_random_points.txt" "plane_inlier_points.txt" "plane.txt"
"line_random_points.txt" "line_inlier_points.txt" "line.txt"

The file "plane.txt" and "line.txt" save the parameters of plane and line.
You can open the xxx_random_points.txt files in [CloudCompare](http://www.cloudcompare.org/main.html);

## License

MtGRansac is released under an [MIT License](https://opensource.org/licenses/MIT).

## Contact

Chalie Wang (CharileV5@qq.com)
