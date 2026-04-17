## Using the dataset generation:

Quick usage:
```bash
python data/dataset_generation.py <perfect|imperfect> <small|medium|large> <count> [-o OUTPUT_DIR] [--seed N] [--extra-wall-removals F]
```

```bash
python data/dataset_generation.py perfect small 8
python data/dataset_generation.py imperfect medium 25 -o data/mazes --extra-wall-removals 0.08
```