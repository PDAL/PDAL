# import numpy
import numpy as np

# import matplotlib stuff and make sure to use the
# AGG renderer.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab

# This only works for Python 3. Use
# StringIO for Python 2.
from io import BytesIO

# The make_plot function will do all of our work. The
# filters.programmable filter expects a function name in the
# module that has at least two arguments -- "ins" which
# are numpy arrays for each dimension, and the "outs" which
# the script can alter/set/adjust to have them updated for
# further processing.
def make_plot(ins, outs):

    # figure position and row will increment
    figure_position = 1
    row = 1

    fig = plt.figure(figure_position, figsize=(6, 8.5), dpi=300)

    for key in ins:
        dimension = ins[key]
        ax = fig.add_subplot(len(ins.keys()), 1, row)

        # histogram the current dimension with 30 bins
        n, bins, patches = ax.hist( dimension, 30,
                                    density=0,
                                    facecolor='grey',
                                    alpha=0.75,
                                    align='mid',
                                    histtype='stepfilled',
                                    linewidth=None)

        # Set plot particulars
        ax.set_ylabel(key, size=10, rotation='horizontal')
        ax.get_xaxis().set_visible(False)
        ax.set_yticklabels('')
        ax.set_yticks((),)
        ax.set_xlim(min(dimension), max(dimension))
        ax.set_ylim(min(n), max(n))

        # increment plot position
        row = row + 1
        figure_position = figure_position + 1

    # We will save the PNG bytes to a BytesIO instance
    # and the nwrite that to a file.
    output = BytesIO()
    plt.savefig(output,format="PNG")

    # a module global variable, called 'pdalargs' is available
    # to filters.programmable and filters.predicate modules that contains
    # a dictionary of arguments that can be explicitly passed into
    # the module by the user. We passed in a filename arg in our `pdal pipeline` call
    if 'filename' in pdalargs:
        filename = pdalargs['filename']
    else:
        filename = 'histogram.png'

    # open up the filename and write out the
    # bytes of the PNG stored in the BytesIO instance
    o = open(filename, 'wb')
    o.write(output.getvalue())
    o.close()


    # filters.programmable scripts need to
    # return True to tell the filter it was successful.
    return True
