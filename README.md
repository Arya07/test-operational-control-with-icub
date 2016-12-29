Operational Control with iCub
=============================

# Prerequisites
By now, you should be an expert of the following components :wink::
- [Cartesian Control](http://wiki.icub.org/iCub/main/dox/html/icub_cartesian_interface.html).
- [Gaze Control](http://wiki.icub.org/iCub/main/dox/html/icub_gaze_interface.html).

# Assignment
We want you to develop a rather simple module that employs the `Cartesian Interface`
and the `Gaze Interface` to accomplish the following tasks:

1. Make iCub **look down at the table**.
2. **Detect the blue ball** in both image planes.
3. Retrieve the **ball position** in the Cartesian domain.
4. Ask iCub to reach for the ball and **make it roll**.

![make-it-roll](/misc/make-it-roll.gif)

## Notes
- To make the blue ball show up within the simulator, you have to turn on the
**objects** flag in the [**`iCub_parts_activation.ini`**](https://github.com/robotology/icub-main/blob/master/app/simConfig/conf/iCub_parts_activation.ini#L28) file.

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/wiki/How-to-complete-assignments)
