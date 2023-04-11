# LiDAR Point-Cloud Statistics

On the basis of data and statistics collected from the bag file, we can make the following observations:
The LiDAR point-cloud has a width and height of (512, 128) and a depth of 9 (x, y, z, intensity, time, reflectivity, ring, ambient, range).

From a bit of research, the closest matching LiDAR sensor that has most likely been used is Ouster OS2. The data sheet for this sensor can be found [here](https://ouster.com/wp-content/uploads/2020/03/OS2-128-Datasheet.pdf).

## Data Statistics

### Valid Points Statistics

- Max possible number points = 512 * 128 = 65536
- Average number of points: 33968
- Minimum number of points: 30683
- Maximum number of points: 44107

### Reflectivity Statistics

- This might be skewed by the fact that the reflectivity is only for the valid x, y, z points, not the full 512 * 128.
- Average reflectivity: 11.033288071795207
- Minimum reflectivity: 0
- Maximum reflectivity: 255

### Intensity Statistics

- Average intensity: 234.65756225585938
- Minimum intensity: 1.0
- Maximum intensity: 17372.0

### Range Statistics

- Looking from the numbers
- Average range: 3456.3175131634966
- Minimum range: 0
- Maximum range: 483343

## Additional Information

- A fixed portion of the LiDAR point-cloud is always blank (not sure why because as per data sheets, the horizontal FOV is 360 degrees).
- Given the number of points seen in the bag file is relatively on the lower side, we can get away with not downsampling the data before processing. However, the max points the sensor can capture (as per data sheet, in 128 channel config) is 2,621,440 points per second, so downsampling might be required when working with real-time data.
- The reflectivity values are not very high, so we can use the intensity values as a proxy for reflectivity.
- For ground plane removal, we can combine the intensity and range values to get a better estimate of the ground plane.
