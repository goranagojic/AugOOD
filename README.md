# AugOOD: Augment images for out-of-distribution robustness evaluation

Image augmentation tool for out-of-distribution (OOD) robustness evaluation implemented in C++ for fast augmentation of 
large image datasets.

OOD robustness evaluation data is generated by applying specified image transformations on input data to get the 
evaluation data. Currently supported transformations include blur (Gaussian and motion blur), and different noise types.
Here you can see some examples of blurred images converted into gifs.

<h3 style="text-align: center;">Gaussian blur (left) and motion blur - arbitrary angle (40°) (right)</h3>
<figure style="display:flex">
    <div>
        <img alt="Gaussian blur" src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMXI2andxNDl3aDg0emdla3Bjb3BlaGZ4M3EycW1kcTl6bDV0dHlqdSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/eY801QMNufzzM9PUaE/giphy.gif"/>
        <img alt="Motion blur - angle" src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExNjVwdWQybW5rM2ZwY2o5emdwcjN5ejUyOHJ5NTVodWp0cDIwMXhuNCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/ennUQoPSo6gYSVJYce/giphy.gif"/>
    </div>
</figure>

<h3><center>Vertical motion blur (left) and horizontal motion blur (right)</center></h3>
<figure style="display: flex">
    <div>
        <img alt="Motion blur - vertical" src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMnI3M3ZmcWd5cGF0YnhvOWx4OWNvdDV4aGN3c3RwYXh0cDk4d3Z1eSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/hS9NsSlmja393Z5x4s/giphy.gif"/>
        <img alt="Motion blur - horizontal" src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExNW4wMTRpbzc2MWt3dTZjb3c1aXh3dDd2Y25uOG1mMGU3c2N4cjNkZyZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/ca3Y9JVrzBFErRS3Rh/giphy.gif"/>
    </div>
</figure>


[//]: # (![Gaussian blur]&#40;https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMXI2andxNDl3aDg0emdla3Bjb3BlaGZ4M3EycW1kcTl6bDV0dHlqdSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/eY801QMNufzzM9PUaE/giphy.gif&#41;)

[//]: # (![Motion blur - 40 degree angle]&#40;https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExNjVwdWQybW5rM2ZwY2o5emdwcjN5ejUyOHJ5NTVodWp0cDIwMXhuNCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/ennUQoPSo6gYSVJYce/giphy.gif&#41;)
[//]: # (![Motion blur - vertical]&#40;https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMnI3M3ZmcWd5cGF0YnhvOWx4OWNvdDV4aGN3c3RwYXh0cDk4d3Z1eSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/hS9NsSlmja393Z5x4s/giphy.gif&#41;)

[//]: # (![Motion blur - horizontal]&#40;https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExNW4wMTRpbzc2MWt3dTZjb3c1aXh3dDd2Y25uOG1mMGU3c2N4cjNkZyZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/ca3Y9JVrzBFErRS3Rh/giphy.gif&#41;)

## Install and Use

### Prerequisites
- **Docker**: Ensure Docker is installed on your system. You can download and install Docker from [here](https://www.docker.com/get-started).
- **Git**: Ensure Git is installed to clone your repository. You can download and install Git from [here](https://git-scm.com/downloads).

### Build the Docker Image
Navigate to the directory where Dockerfile is located. Run the following command to build the Docker image from your Dockerfile:

```
docker build -t augood .
```

### Run the Docker Container
Once the image is built, you can run a container based on this image. In the case of this tutorial, use the following command:

```
docker run -it --name augoodcontainer augood
```

`--name` assigns a name to your container, you can change it to the arbitrary name

Now you should be in the container's `\home` directory where you can build the tool using `cmake` and `make` utilities like this:

```bash
cmake . && make
```

This command should produce executable `mod` that is used to generate augmented images.


## How to use
Here is an example of how to use the tool to augment all images from `sampleInput` directory with blur type transformations and 
kernel sizes [3, 5, 7, 9]. Augmented images are stored to `sampleOutput` directory.
```bash
./mod -I sampleInput/ -O sampleOutput --startKernel 3 --endKernel 9 --stepKernel 2 --runOptions "BLUR"
```
All options can be listed with `./mod --help`.
 
### Transformations
At the moment, different blur and noise transformations are supported in the tool. You can see a complete list in the table.
Column `runOptions` contains string that is to be passed to `--runOptions` parameter to apply transformation described in the
description column.

| `runOptions` arg     	 | Type  	      | Description                                                   	   |
|------------------------|--------------|-------------------------------------------------------------------|
| `"GAUSS_UNI_BLUR"`   	 | Blur  	      | Gaussian blur, simulates camera defocus.                      	   |
| `"MOT_BLUR_C"`       	 | Blur  	      | Motion blur, arbitrary angle. Simulates camera/object motion. 	   |
| `"MOT_BLUR_H"`       	 | Blur  	      | Motion blur, horizontal.                                      	   |
| `"MOT_BLUR_V"`       	 | Blur  	      | Motion blur, vertical.                                        	   |
| `"MOT_BLUR"`       	   | Blur  	      | Expands to all types of motion blur.                                   |
| `"BLUR"` 	             | Blur 	       | Expands to Gaussian blur and all types of motion blur.         	  |
| `"GAUSS_NOISE_MONO"` 	 | Noise 	      | Gaussian noise, same noise intensity on all channels.         	   |
| `"GAUSS_NOISE"`      	 | Noise 	      | Gaussian noise, different noise intensity on all channels.    	   |
| `"GAUSS_NOISE_R"`    	 | Noise 	      | Gaussian noise, applied on R channel.                         	   |
| `"GAUSS_NOISE_G"`    	 | Noise 	      | Gaussian noise, applied on G channel.                         	   |
| `"GAUSS_NOISE_B"`    	 | Noise 	      | Gaussian noise, applied on B channel.                         	   |
| `"GAUSS_NOISE_YUV"`  	 | Noise 	      | Gaussian noise applied on image transformed to YUV space.     	   |
| `"GAUSS_NOISE_FREQ"` 	 | Noise 	      | Gaussian noise applied on image in frequency domain.          	   |
| `"NOISE"` 	            | Noise 	      | Expands to all noise types.          	                            |
| `"ALL"` 	              | Blur+Noise 	 | Expands to `BLUR` + `NOISE`.          	                           |


## Cite

When using the tool please cite the paper [Robustness of deep learning methods for ocular fundus segmentation: Evaluation of blur sensitivity](https://onlinelibrary.wiley.com/doi/abs/10.1002/cpe.6809).

Latex citation:
```latex
@article{petrovic_2022_oodrobustnesseval,
  title={Robustness of deep learning methods for ocular fundus segmentation: Evaluation of blur sensitivity},
  author={Petrović, Veljko B and Gojić, Gorana and Dragan, Dinu and Gajić, Dušan B and Horvat, Nebojša and Turović, Radovan and Oros, Ana},
  journal={Concurrency and Computation: Practice and E sexperience},
  volume={34},
  number={14},
  pages={e6809},
  year={2022},
  publisher={Wiley Online Library}
}
```

## Authors
- Veljko Petrović (@pveljko)
- Gorana Gojić (@goranagojic)
