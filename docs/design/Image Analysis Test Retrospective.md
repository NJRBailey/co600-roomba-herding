# Image analysis tests retrospective

As stated at the start of our project, we did not follow the principles of Test-Driven Development due to the time commitment it would require. Despite this, tests were eventually written for the image analysis functions. In theory, these should have been useful to show us if functionality broke after editing, and also assure us that the code worked as intended. However, the reality is that the tests were generally unhelpful, and took up a few days of development which could have been used elsewhere. We have identified two main reasons for the tests being unhelpful.

## Tests were not helpful

Tests did not fully predict real behaviour, even in the simulator. Despite gathering a variety of screenshots from the simulator and the real drone camera, errors would still occur during live operation. Capturing the exact frame which caused a crash was tedious work, and the fixes applied seemed to only fix that frame, never fully reducing the presence of a particular error. Frames which caused crashes rarely had an obvious crash-causing feature, meaning that finding a robust fix to an error was usually impossible.

Despite these criticisms, some unimplemented solutions have been suggested, particularly the use of a different line equation (as described in the future improvements section TODO). These suggestions were informed by findings made during testing.

## Tests added too late

Tests were added in February. By this time, the code to recognise markers had been written and modified multiple times over a period of four months. This meant that any changes would affect a large amount of image analysis code, which would require more time, and made changes (which may or may not have improved reliability) unattractive to implement.

If the tests had been added alongside the code implementation, they would likely have helped inform the design and theory of the image analysis code, so that major changes would not have to be considered later when most of the code had been written. However, due to the unreliability of the tests (as discussed in the previous point) it is hard to say for sure whether this would have been helpful either.