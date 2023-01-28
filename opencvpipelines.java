package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
public class opencvpipelines extends OpenCvPipeline{
        Mat mat = new Mat();

        public byte[][][] image;
        @Override
        public Mat processFrame(Mat input) {
                Mat c = input.clone();
                int pixelsCounter = 0;
                ArrayList<ArrayList<Double>> pixels = new ArrayList<>();
                for (int i = 0; i < c.height(); i++) {
                        for (int j = 0; j < c.width(); j++) {
                                pixelsCounter++;
                                ArrayList<Double> tmp = new ArrayList<>();
                                tmp.add(c.get(i, j)[0]);
                                tmp.add(c.get(i, j)[1]);
                                tmp.add(c.get(i, j)[2]);
                                pixels.add(tmp);
                        }
                }
                byte[][][] p = (byte[][][]) pixels.toArray();
                image = p;
                return input;
        }

        public byte[][][] get_pixels(){
                return image;
        }
        }
