# Toy Pose Graph Optimization

Useful resources:
 [https://kusemanohar.wordpress.com/2017/04/29/howto-pose-graph-bundle-adjustment/](https://kusemanohar.wordpress.com/2017/04/29/howto-pose-graph-bundle-adjustment/).

[https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam](https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam)

### Requirements

```
$ sudo apt install libceres-dev libeigen3-dev
```

### Build

```
mkdir build
cd build
cmake ..
make
```

This should produce an executable `example_2d`. Run this executable from build folder.

### Run Executable
```
./example_2d
```

This executable reads file `../data/M3500.g2o` and produces
`../data/init_nodes.txt` and `../data/after_opt_nodes.txt`

### Visualize Results
We have provided a python script to visualize the results. The text files to supply should contain lines as : `id x y theta` representing every vertex.

```
python ../util/plot_results.py --initial_poses ../data/init_nodes.txt --optimized_poses ../data/after_opt_nodes.txt
```

