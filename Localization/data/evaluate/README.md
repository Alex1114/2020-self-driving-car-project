# Evalation code

## Requirement

### numpy
```
pip3 install numpy
```

## Execution
There are some arguments you need to set:
  -g, --ground-truth: The ground truth csv file you want to evaluation
  -r, --result: Your result csv file you want to evaluation
                        
For example:
```
python3 evaluate.py -g itri_200_gt.csv itri_700.csv itri_1400_gt.csv -r path200.csv path700.csv path1400.csv
```

## Help
```
python3 evaluate.py -h
```
