import os
import sys
import logging
import gdown
from zipfile import ZipFile

dataset_url = 'https://drive.google.com/u/1/uc?id=1Y5ietDYpmXSJXaqvFg3BCb3p_-Q_HlRp&export=download'
dataset_name = 'data'

gdown.download(dataset_url, output=dataset_name + '.zip', quiet=False)
zip1 = ZipFile(dataset_name + '.zip')
zip1.extractall(dataset_name)
zip1.close()
os.remove(dataset_name + '.zip')

print("Finished downloading Nuscenes data.") 