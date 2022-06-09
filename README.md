# building-and-tracking-a-road-model

[![Build Status](https://app.travis-ci.com/alechh/building-and-tracking-a-road-model.svg?token=nkfHMH8bU4FvD1venBAz&branch=master)](https://app.travis-ci.com/alechh/building-and-tracking-a-road-model)

[![Codacy Badge](https://app.codacy.com/project/badge/Grade/f28509be7b62493296897c9ef2638864)](https://www.codacy.com/gh/alechh/building-and-tracking-a-road-model/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=alechh/building-and-tracking-a-road-model&amp;utm_campaign=Badge_Grade)

Repository for solving the problem of building a road model online and then tracking it

## Requirements
  *  CMake (>= 3.20)
  *  OpenCV (>= 4.4.0)

## Build
```bash 
mkdir build && cd build
cmake ..
make
```

## Run main
```bash
 ./building-and-tracking-a-road-model
```

## Run unit-tests
```bash
cd Tests
./tests
```
