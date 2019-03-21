# Future improvements

## Improvements to current functionality

### Explore use of alternative equation of a line

The current image analysis code frequently uses the equation of a line in the form ‘y = mx + c’. This means calculations can fail when m is calculated as infinity, and there is a lot of additional code to account for these cases. It might be possible to simplify the code by using the equation of a line in the form rho = xCosA + ySinA. This is the equation of a line used in the Hough transform, and is used to remove the need to deal with ‘m’.

Using this form of the line equation could also improve detection reliability, as currently the detection can be inconsistent due to mathematical rounding errors. Using this equation may reduce the amount of rounding which takes place and improve reliability.

### Use all currently-implemented noise reduction techniques

There are three noise reduction techniques currently implemented, of which only one was actually used. The other two are useful, but have not been integrated properly with the running code. This could be integrated into either a single, robust technique, or each technique could be picked automatically depending on the conditions in the frame.

The segmentation of the image could also be improved. Currently a single set of customisable-size regions can be set, but once a region is created, it is only removed or kept. This process could be iterative, with large regions used to remove large areas of unnecessary pixels, and then smaller regions used to remove smaller areas, and more accurately define the useful areas.

### Implement further noise and feature reduction techniques

Further noise reduction techniques were identified, but were not implemented due to their lack of relevance to the simulator. Using histogram equalisation might allow for a greater distinction between the light and dark areas of the markers, particularly under natural light where the dark ink of the markers can end up being more of a mid-dark blue than a dark black.

Standard noise reduction filters (such as a Gaussian filter) could be applied, if necessary, to reduce noise from a real camera.

## Improvements to future functionality

### Use hazard tape and/or GPS for arena boundary

The original plan for the boundaries, when we were still developing for the real world, was to use hazard tape. This would have allowed, in our minds, for more reliable detection in real life. This plan was dropped when focus switched purely to the simulator, but the idea is still viable for future development.

Alternatively, a drone with a GPS module could be used. This would be more directly applicable to a real search and rescue operation, and would save image processing time. This in turn would allow for more intensive image analysis to identify the lost robot. It would also be important to free up resources to perform the 2nd suggested future functionality.

### Alternative/No markers

Not using markers would be the most advanced change. Depending on usage, this could be a neural network which has been trained to identify objects (e.g. Roombas) or pure image analysis (e.g. identifying a green shirt in a dark blue ocean). With different drone equipment, different camera types could also be used (e.g. a thermal camera would spot a warm body in a cold sea very easily).
