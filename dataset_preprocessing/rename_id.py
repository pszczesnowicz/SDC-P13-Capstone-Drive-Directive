import csv

read_path = '/Users/skunkworks/programming/SDCND/Projects/SDC-P13-Capstone-Drive-Directive/labels.csv'

new_labels = []

with open(read_path, 'r') as labels_reader:
    labels = csv.reader(labels_reader, delimiter=',')

    for row in labels:
        if row[2] == 'red':
            row[1] = '1'

        elif row[2] == 'yellow':
            row[1] = '2'

        elif row[2] == 'green':
            row[1] = '3'

        new_labels.append(row)

write_path = '/Users/skunkworks/programming/SDCND/Projects/SDC-P13-Capstone-Drive-Directive/labels_new.csv'

with open(write_path, 'w') as labels_writer:
    labels = csv.writer(labels_writer)
    labels.writerows(new_labels)
