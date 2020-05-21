import os
import sys
import logging
import gdown
from zipfile import ZipFile

dataset_url = 'https://drive.google.com/u/1/uc?id=1KonoZkKsxC3CZMnlLy0OvakR6qB7QvFs&export=download'
dataset_name = 'data'

gdown.download(dataset_url, output=dataset_name + '.zip', quiet=False)
zip1 = ZipFile(dataset_name + '.zip')
zip1.extractall(dataset_name)
zip1.close()
os.remove(dataset_name + '.zip')

print("Finished downloading ITRI data.") 