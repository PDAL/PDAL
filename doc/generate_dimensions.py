import csv
import json
import os

# Get the directory of conf.py
conf_dir = os.path.dirname(os.path.abspath(__file__))
json_path = os.path.join(conf_dir, '..', 'pdal', 'Dimension.json')
csv_path = os.path.join(conf_dir, 'dimension-table.csv')
print(json_path)
print(csv_poath)

data = open(json_path, 'r').read()

data = json.loads(data)['dimensions']

output = []
for dim in data:
    output.append([dim['name'], dim['type'], dim['description']])

output = sorted(output,key=lambda x: x[0])

with open(csv_path, 'w') as fp:
    a = csv.writer(fp, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
    a.writerows(output)