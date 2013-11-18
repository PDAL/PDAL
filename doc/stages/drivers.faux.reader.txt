.. _drivers.faux.reader:

drivers.faux.reader
===================

The "**faux reader**" is used for testing pipelines. It does not read from a file or database, but generates synthetic data to feed into the pipeline.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.text.writer">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="drivers.faux.reader">
        <Option name="bounds">([0,1000000],[0,1000000],[0,100])</Option>
        <Option name="num_points">10000</Option>
        <Option name="mode">random</Option>
      </Reader>
    </Writer>
  </Pipeline>


Options
-------

bounds
  What spatial extent should points be generated within? Text string of the form "([xmin,xmax],ymin,ymax],[zmin,zmax])" [Required]

num_points
  How many synthetic points to generate before finishing? [Required]
  
mode
  How to generate synthetic points. One of "constant" (repeat single value), "random" (random values within bounds), or "ramp" (steadily increasing values within the bounds). [Required]
  
  

  
