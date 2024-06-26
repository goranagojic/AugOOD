FROM opencvcourses/opencv-docker
# update & upgrade ubuntu
RUN apt-get update -y && apt-get upgrade -y
# install compilation tools
RUN apt-get install wget cmake make g++ -y
WORKDIR /home
# copy the content of a pictureModifier directory to a container
COPY . ${WORKDIR}
# install boost library
#    - installs boost headers and 
#    - program options library
WORKDIR /usr/local
RUN wget https://boostorg.jfrog.io/artifactory/main/release/1.75.0/source/boost_1_75_0.tar.bz2 && \
    tar --bzip2 -xf boost_1_75_0.tar.bz2 && \
    rm boost_1_75_0.tar.bz2 && \
    cd boost_1_75_0 && \
    bash bootstrap.sh && \
    ./b2 install --with-program_options
# build executable files in /home/build directory
WORKDIR /home
RUN mkdir build && cd build && cmake .. && make
RUN mkdir sampleInput sampleOutput && mv sampleCode/boy.jpg sampleInput && rm -r sampleCode