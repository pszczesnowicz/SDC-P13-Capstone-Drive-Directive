#adjust width and height
import csv
with open('labels.csv') as csvfile:
    reader = csv.DictReader(csvfile)

    #reoder the columns: done
    for row in reader:
        row['id']=800 #width
        row['name']=600 #height
        row['new']=row['yMax'] #ymax
        row['yMax']=row['xMax'] #xmax
        row['xMax']=row['xMin'] #xmin
        row['xMin']='red' #class
        #row['yMin']=240 #ymin already correct
        print(row['image'], row['id'], row['name'], row['xMin'], row['xMax'], row['yMin'], row['yMax'], row['new'])
        # change header manually to filename,width,height,class,xmin,ymin,xmax,ymax
        #ToDO:rename headers/columns:
        """
        reader.rename(columns={
                 'image': 'filename',
                 'id': 'width',
                 'name': 'height',
                 'xMin': 'class',
                 'xMax': 'xmin',
                 'yMin': 'ymin',
                 'yMax': 'xmax',
                 'new': 'ymax'}, inplace=True)




    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    writer.writerow({'first_name': 'Baked', 'last_name': 'Beans'})
    writer.writerow({'first_name': 'Lovely', 'last_name': 'Spam'})
    writer.writerow({'first_name': 'Wonderful', 'last_name': 'Spam'})
"""
