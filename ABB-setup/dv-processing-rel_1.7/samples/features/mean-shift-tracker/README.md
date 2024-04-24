## Mean-shift tracker

The following code sample shows the use of mean-shift tracker implementation to find and track events on a sample
recording that can be downloaded at the following
[link](https://s3.eu-central-1.amazonaws.com/release.inivation.com/datasets/mean_shift_sample.aedat4).

Note that tracks are initialized detecting blobs on an accumulated event-image. The algorithm does not track blobs
appearing after the first iteration, since track reinitialization is not part of it.
