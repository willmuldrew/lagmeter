Lagmeter
--------

Hardware works by itself and computes display lag from a simulated HID event to a dark to/from light transition on the screen.  Results are displayed on OLED screen.

Different modes can be selected by holding down the button to cycle through keypress (x), alt-tab, alt-esc, mouse click etc...

Optionally, you can run some software on the target machine:
 * lagcap.py - captures packets on an interface and pulls light sensor results from arduino. Writes both into csvs in your temp dir
 * LagWinApp.exe - super minimal windows app that cycles from black-white-black with keypresses (x).  Also logs timestamped WM_PAINT, WM_KEYDOWN events to the tempdir
 * ProcessResults.ipynb jupyter notebook - reads in arduino output, pcap and LagWinApp output from disk and plots some results


Getting started
---------------

1. Upload arduino sketch to your Leonardo (or clone)
1. Run lagcap.py locally in a command prompt (e.g. I use a anaconda prompt in a conda environment 
with the relevant deps - pyshark, pyserial, pandas)  Strangely, no admin prompt required despite serial and pcaps
1. Point sensor somewhere on the screen (e.g. just past the cursor on a text prompt - will depend on the selected mode)
1. Hit button
1. Results are written to tmp dir and can be viewed with the ProcessResults jupyter notebook


Building LagWinApp
------------------

If you want a really minimal baseline app you can build and run LagWinApp and use that as your test target. It will drop some timestamped window message logs in the temp dir which can be plotted against other data.

I've build this wtih Visual Studio 2019.
i

Dependencies
------------

pyshark, pyserial and pandas
