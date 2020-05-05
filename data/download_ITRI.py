import os
import sys
import logging
import gdown
from zipfile import ZipFile

dataset_url = 'https://drive.google.com/uc?id=1BdETu4DW9jUzhr-Prvf1ykh0NlhrtyIs'
dataset_name = 'ITRI'
if not os.path.isdir(dataset_name):
    gdown.download(dataset_url, output=dataset_name + '.zip', quiet=False)
    zip1 = ZipFile(dataset_name + '.zip')
    zip1.extractall(dataset_name)
    zip1.close()

print("Finished downloading ITRI data.") 