from pathlib import Path
import urllib.request
import pdal
import os

class Suite:

    laz_file = Path('autzen.laz')
    las_file = Path('autzen.las')

    def setup(self):
        # see if we already have autzen
        if not self.laz_file.exists():
            response = urllib.request.urlopen("https://github.com/PDAL/data/raw/main/autzen/autzen.laz")
            with open("autzen.laz", "wb") as f:
                f.write(response.read())
        
        # now check for presence of las file
        if not self.las_file.exists():
            pipeline = pdal.Reader.laz(
                filename = os.fsdecode(self.laz_file),
            )
            pipeline |= pdal.Writer.las(
                filename = os.fsdecode(self.las_file)
            )
            pipeline.execute()

    # las -> las
    def convert_las_to_las(self):
        pipeline = pdal.Reader.las(
            filename = os.fsdecode(self.las_file)
        )
        pipeline |= pdal.Writer.las(
            filename = "temp.las"
        )
        pipeline.execute()

    # las -> laz
    def convert_las_to_laz(self):
        pipeline = pdal.Reader.las(
            filename = os.fsdecode(self.las_file)
        )
        pipeline |= pdal.Writer.laz(
            filename = "temp.laz"
        )
        pipeline.execute()

    # laz -> laz
    def convert_laz_to_laz(self):
        pipeline = pdal.Reader.laz(
            filename = os.fsdecode(self.laz_file)
        )
        pipeline |= pdal.Writer.laz(
            filename = "temp.laz"
        )
        pipeline.execute()
    
    # laz -> copc
    def laz_to_copc(self):
        pipeline = pdal.Reader.laz(
            filename = os.fsdecode(self.laz_file)
        )
        pipeline |= pdal.Writer.laz(
            filename = "temp.copc.laz"
        )
        pipeline.execute()
