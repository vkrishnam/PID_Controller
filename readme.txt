## Main technique involves the Twiddle of the PID coefficients
The main approach taken to make the car drive all through the track to:
1. First arrive at the initial values of {0.1, 0.0005, 10} after trail-and-error to make the car run straight
2. Device a twiddle scheme every 200 steps, with initial dp = {0.1, 0.0001, 1}. This scheme resets itself at the end of every 200 steps to adapt to the new section of
       the track.
