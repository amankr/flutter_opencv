package com.mulgundkar.opencv.core;

import android.annotation.SuppressLint;

import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;

import java.util.ArrayList;
import java.util.List;

import androidx.annotation.NonNull;
import io.flutter.Log;
import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.MethodChannel.MethodCallHandler;
import io.flutter.plugin.common.MethodChannel.Result;
import io.flutter.plugin.common.PluginRegistry.Registrar;

import static org.opencv.core.CvType.CV_8UC1;

/**
 * OpenCV4Plugin
 */
public class CVCore {

    @SuppressLint("MissingPermission")
    public byte[] cvtColor(byte[] byteData, int outputType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Convert the image color
            Imgproc.cvtColor(src, dst, outputType);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] blur(byte[] byteData, ArrayList kernelSize, ArrayList anchorPoint, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            Size size = new Size((double) kernelSize.get(0), (double) kernelSize.get(1));
            Point point = new Point((double) anchorPoint.get(0), (double) anchorPoint.get(1));
            // Convert the image to Gray
            Imgproc.blur(src, dst, size, point, borderType);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] gaussianBlur(byte[] byteData, ArrayList kernelSize, double sigmaX) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            Size size = new Size((double) kernelSize.get(0), (double) kernelSize.get(1));
            // Convert the image to Gray
            Imgproc.GaussianBlur(src, dst, size, sigmaX);
            Mat rotate = new Mat();
            Core.rotate(dst, rotate, Core.ROTATE_90_CLOCKWISE);
            System.out.println("Rotating");
            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", rotate, matOfByte);
            byteArray = matOfByte.toArray();
            System.out.println("Source: " + byteData.length);
            System.out.println("Output: " + byteArray.length);
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] medianBlur(byte[] byteData, int kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Convert the image to Gray
            Imgproc.medianBlur(src, dst, kernelSize);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] bilateralFilter(byte[] byteData, int diameter, int sigmaColor, int sigmaSpace, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Convert the image to Gray
            Imgproc.bilateralFilter(src, dst, diameter, sigmaColor, sigmaSpace, borderType);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] boxFilter(byte[] byteData, int outputDepth, ArrayList kernelSize, ArrayList anchorPoint, boolean normalize, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            Size size = new Size((double) kernelSize.get(0), (double) kernelSize.get(1));
            Point point = new Point((double) anchorPoint.get(0), (double) anchorPoint.get(1));

            // Convert the image to Gray
            Imgproc.boxFilter(src, dst, outputDepth, size, point, normalize, borderType);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] sqrBoxFilter(byte[] byteData, int outputDepth, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            Size size = new Size((double) kernelSize.get(0), (double) kernelSize.get(1));

            // Convert the image to Gray
            Imgproc.sqrBoxFilter(src, dst, outputDepth, size);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] filter2D(byte[] byteData, int outputDepth, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Creating kernel matrix
            Mat kernel = Mat.ones((int) kernelSize.get(0), (int) kernelSize.get(1), CvType.CV_32F);

            for(int i = 0; i<kernel.rows(); i++) {
                for(int j = 0; j<kernel.cols(); j++) {
                    double[] m = kernel.get(i, j);

                    for(int k = 1; k<m.length; k++) {
                        m[k] = m[k]/(2 * 2);
                    }
                    kernel.put(i,j, m);
                }
            }
            // Convert the image to Gray
            Imgproc.filter2D(src, dst, outputDepth, kernel);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] dilate(byte[] byteData, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Preparing the kernel matrix object
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                    new  Size(((int) kernelSize.get(0)*(int) kernelSize.get(1)) + 1, ((int) kernelSize.get(0)*(int) kernelSize.get(1))+1));

            // Convert the image to Gray
            Imgproc.dilate(src, dst, kernel);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] erode(byte[] byteData, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Preparing the kernel matrix object
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                    new  Size(((int) kernelSize.get(0)*(int) kernelSize.get(1)) + 1, ((int) kernelSize.get(0)*(int) kernelSize.get(1))+1));

            // Convert the image to Gray
            Imgproc.erode(src, dst, kernel);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] morphologyEx(byte[] byteData, int operation, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Creating kernel matrix
            Mat kernel = Mat.ones((int) kernelSize.get(0),(int) kernelSize.get(0), CvType.CV_32F);

            // Morphological operation
            Imgproc.morphologyEx(src, dst, operation, kernel);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] pyrUp(byte[] byteData, ArrayList kernelSize, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Size of the new image
            Size size = new Size((int) kernelSize.get(0), (int) kernelSize.get(1));

            // pyrUp operation
            Imgproc.pyrUp(src, dst, size, borderType);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] pyrDown(byte[] byteData, ArrayList kernelSize, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Size of the new image
            Size size = new Size((int) kernelSize.get(0), (int) kernelSize.get(1));

            // pyrDown operation
            Imgproc.pyrDown(src, dst, size, borderType);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] pyrMeanShiftFiltering(byte[] byteData, double spatialWindowRadius, double colorWindowRadius) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // pyrMeanShiftFiltering operation
            Imgproc.pyrMeanShiftFiltering(src, dst, spatialWindowRadius, colorWindowRadius);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] threshold(byte[] byteData, double thresholdValue, double maxThresholdValue, int thresholdType) {
        byte[] byteArray = new byte[0];
        try {
            Mat srcGray = new Mat();
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);
            // Convert the image to Gray
            Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);

            // Thresholding
            Imgproc.threshold(srcGray, dst, thresholdValue, maxThresholdValue, thresholdType);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] adaptiveThreshold(byte[] byteData, double maxValue, int adaptiveMethod, int thresholdType, int blockSize, double constantValue) {
        byte[] byteArray = new byte[0];
        try {
            Mat srcGray = new Mat();
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Convert the image to Gray
            Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);

            // Adaptive Thresholding
            Imgproc.adaptiveThreshold(srcGray, dst, maxValue, adaptiveMethod, thresholdType, blockSize, constantValue);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] copyMakeBorder(byte[] byteData, int top, int bottom, int left, int right, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // copyMakeBorder operation
            Core.copyMakeBorder(src, dst, top, bottom, left, right, borderType);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] sobel(byte[] byteData, int depth, int dx, int dy) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Sobel operation
            Imgproc.Sobel(src, dst, depth, dx, dy);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] scharr(byte[] byteData, int depth, int dx, int dy) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Scharr operation
            Imgproc.Scharr(src, dst, depth, dx, dy);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] laplacian(byte[] byteData, int depth) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Laplacian operation
            Imgproc.Laplacian(src, dst, depth);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] distanceTransform(byte[] byteData, int distanceType, int maskSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // distanceTransform operation
            Imgproc.distanceTransform(src, dst, distanceType, maskSize);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] resize(byte[] byteData, ArrayList outputSize, double fx, double fy, int interpolation) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // Size of the new image
            Size size = new Size((int) outputSize.get(0), (int) outputSize.get(1));

            // resize operation
            Imgproc.resize(src, dst, size, fx, fy, interpolation);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] applyColorMap(byte[] byteData, int colorMap) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // resize operation
            Imgproc.applyColorMap(src, dst, colorMap);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] canny(byte[] byteData, double threshold1, double threshold2) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);

            // resize operation
            Imgproc.Canny(src, dst, threshold1, threshold2);

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] houghCircles(byte[] byteData, int method, double dp, double minDist, double param1, double param2, int minRadius, int maxRadius, int centerWidth, String centerColor, int circleWidth, String circleColor) {
        byte[] byteArray = new byte[0];
        try {
            Mat circles = new Mat();
            // Decode image from input byte array
            Mat input = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);
            //Imgproc.medianBlur(input, input, 5);
            // resize operation
            Imgproc.HoughCircles(input, circles, method, dp, minDist, param1, param2, minRadius, maxRadius);

            if (circles.cols() > 0) {
                for (int x=0; x < (circles.cols()); x++ ) {
                    double circleVec[] = circles.get(0, x);

                    if (circleVec == null) {
                        break;
                    }

                    Point center = new Point((int) circleVec[0], (int) circleVec[1]);
                    int radius = (int) circleVec[2];
                    System.out.println("centerColor: " + centerColor);
                    System.out.println("circleColor: " + circleColor);

                    Imgproc.circle(input, center, 3, new Scalar(Integer.valueOf(centerColor.substring( 1, 3 ), 16 ), Integer.valueOf(centerColor.substring( 3, 5 ), 16 ), Integer.valueOf(centerColor.substring( 5, 7 ), 16 )), centerWidth);
                    Imgproc.circle(input, center, radius, new Scalar(Integer.valueOf(circleColor.substring( 1, 3 ), 16 ), Integer.valueOf(circleColor.substring( 3, 5 ), 16 ), Integer.valueOf(circleColor.substring( 5, 7 ), 16 )), circleWidth);
                    System.out.println(x+"th circle");
                }
            }
            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", input, matOfByte);
            byteArray = matOfByte.toArray();
//            System.out.println("OUT: " + dst);
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] perspectiveTransformation(String imagePath, ArrayList points, int frameWidth, int frameHeight){
        byte[] byteArray = new byte[0];
        final int pointSize = 8;
        try{
            if(points.size() != pointSize)
                throw new Exception("points should be 8");

            Mat src = Imgcodecs.imread(imagePath);
            int imageHeight = src.rows();
            int imageWidth = src.cols();
            double X_factor = (double)imageWidth/(double)frameWidth;
            double Y_factor = (double)imageHeight/(double)frameHeight;

            System.out.println("OpenCV Rect Matrix");
            Point tl = new Point((double)points.get(0) * X_factor, (double)points.get(1) * Y_factor);
            Point tr = new Point((double)points.get(2) * X_factor, (double)points.get(3) * Y_factor);
            Point br = new Point((double)points.get(4) * X_factor, (double)points.get(5) * Y_factor);
            Point bl =new Point((double)points.get(6) * X_factor, (double)points.get(7) * Y_factor);
            List<Point> rectPoints = new ArrayList<Point>();
            rectPoints.add(tl);
            rectPoints.add(tr);
            rectPoints.add(br);
            rectPoints.add(bl);

            double widthA = Math.sqrt(((br.x-bl.x) * (br.x-bl.x)) + ((br.y - bl.y) * (br.y - bl.y)));
            double widthB = Math.sqrt(((tr.x-tl.x) * (tr.x-tl.x)) + ((tr.y - tl.y) * (tr.y - tl.y)));
            int maxWidth = Math.max((int)widthA, (int)widthB);

            double heightA = Math.sqrt(((tr.x-br.x) * (tr.x-br.x)) + ((tr.y-br.y) * (tr.y-br.y)));
            double heightB = Math.sqrt(((tl.x-bl.x) * (tl.x-bl.x)) + ((tl.y-bl.y) * (tl.y-bl.y)));
            int maxHeight = Math.max((int)heightA, (int)heightB);

            List<Point> dstPoints = new ArrayList<Point>();
            dstPoints.add(new Point(0, 0));
            dstPoints.add(new Point(maxWidth-1, 0));
            dstPoints.add(new Point(maxWidth-1, maxHeight-1));
            dstPoints.add(new Point(0, maxHeight-1));

            Mat rect = Converters.vector_Point2f_to_Mat(rectPoints);
            Mat dst = Converters.vector_Point2f_to_Mat(dstPoints);

            Mat M = Imgproc.getPerspectiveTransform(rect, dst);
            System.out.println("OpenCV Perspective Matrix "+M.rows()+" "+M.cols()+" "+M);

            Mat wrapped = new Mat();


            Imgproc.warpPerspective(src, wrapped, M, new Size(maxWidth, maxHeight));
            System.out.println("OpenCV warpPerspective ");
            MatOfByte matOfByte = new MatOfByte();
            Imgcodecs.imencode(".jpg", wrapped, matOfByte);
            byteArray = matOfByte.toArray();
            System.out.println("OpenCV Result "+byteArray.length + " "+wrapped.rows()+ " "+wrapped.cols());
        }catch (Exception e){
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] rotateLeft(byte[] byteData){
        byte[] byteArray = new byte[0];
        try{
            Mat dst = new Mat();
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);
            Core.rotate(src, dst, Core.ROTATE_90_COUNTERCLOCKWISE);
            MatOfByte matOfByte = new MatOfByte();
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        }catch (Exception e){
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] rotateRight(byte[] byteData){
        byte[] byteArray = new byte[0];
        try{
            Mat dst = new Mat();
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);
            Core.rotate(src, dst, Core.ROTATE_90_CLOCKWISE);
            MatOfByte matOfByte = new MatOfByte();
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        }catch (Exception e){
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] changeBrightnessAndContrast(byte[] byteData, double brightness, double contrast){
        byte[] byteArray = new byte[0];
        try{
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);
            Mat dst = new Mat(src.rows(), src.cols(), src.type());
            src.convertTo(dst, -1, contrast, brightness);
            MatOfByte matOfByte = new MatOfByte();
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        }catch (Exception e){
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] greyScale(byte[] byteData) {
        byte[] byteArray = new byte[0];
        try{
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);
            Mat dst = new Mat(src.rows(), src.cols(), src.type());
            Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGR2GRAY);
            MatOfByte matOfByte = new MatOfByte();
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        }catch (Exception e){
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] blackAndWhite1(byte[] byteData) {
        byte[] byteArray = new byte[0];

        try {
            System.out.println("blackAndWhite1");
            return adaptiveThreshold(byteData,225,
                    Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, 0, 15, 12);
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }


    @SuppressLint("MissingPermission")
    public byte[] blackAndWhite2(byte[] byteData) {
        byte[] byteArray = new byte[0];

        try {
            System.out.println("blackAndWhite1");
            return threshold(byteData,190,
                    255, 0);
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }


    @SuppressLint("MissingPermission")
    public byte[] autoEnhance(byte[] byteData) {
        float clipHistPercent= 0.06f;
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);
            Size size = new Size((double) 3d, 3d);
            // Convert the image to Gray
            //Imgproc.GaussianBlur(src, src, size, 0);

            int histSize = 256;
            double alpha, beta;
            double minGray = 0, maxGray = 0;
            System.out.println(CV_8UC1);
            System.out.println(CvType.CV_8UC3);
            System.out.println(CvType.CV_8UC4);

            System.out.println(src.type());

            //to calculate grayscale histogram
            Mat gray = new Mat();
            if (src.type() == CV_8UC1) gray = src;
            if (src.type() == CvType.CV_8UC3) Imgproc.cvtColor(src, gray, Imgproc.COLOR_BGR2GRAY);
            if (clipHistPercent == 0)
            {
                // keep full available range
                Core.MinMaxLocResult d = Core.minMaxLoc(gray);
                minGray = d.minVal;
                maxGray = d.maxVal;
            }else{
                Mat hist = new Mat(); //the grayscale histogram
                Mat mask = new Mat();
                MatOfFloat histRange = new MatOfFloat(0f, 256f);
                MatOfInt histSize_ = new MatOfInt(256);
                MatOfInt channels = new MatOfInt(0);
                ArrayList<Mat> listOfMat = new ArrayList<>();
                listOfMat.add(gray);
                boolean accumulate = false;
                Imgproc.calcHist(listOfMat, channels, mask, hist, histSize_, histRange, accumulate);

                double[] accumulator;
                accumulator = new double[histSize];
                accumulator[0] = hist.get(0,0)[0];
                for (int i = 1; i < histSize; i++){
                    accumulator[i] = accumulator[i - 1] + hist.get(i,0)[0];
                }
                // locate points that cuts at required value
                double max = accumulator[histSize-1];
                clipHistPercent *= (max / 100.0); //make percent as absolute
                clipHistPercent /= 2.0; // left and right wings
                // locate left cut
                minGray = 0;
                while (accumulator[(int)minGray] < clipHistPercent)
                    minGray++;

                // locate right cut
                maxGray = histSize - 1;
                while (accumulator[(int)maxGray] >= (max - clipHistPercent))
                    maxGray--;
            }
            // current range
            //double inputRange = maxGray - minGray;
            double inputRange = maxGray - minGray;
            alpha = (histSize - 1) / inputRange;   // alpha expands current range to histsize range
            beta = minGray * alpha;             // beta shifts current range so that minGray will go to 0
            //alpha = 3.072289156626506;
            //beta = -144.3975903614458;

            // Apply brightness and contrast normalization
            // convertTo operates with saurate_cast
            src.convertTo(dst, -1, alpha, beta);
            // TODO add B/W mask on top

            System.out.println("AutoEnhance completed");

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    private MatOfPoint2f orderPointsClockwise(MatOfPoint2f screenCnt2f) {
        System.out.println(screenCnt2f.dump());

        List<Point> points = screenCnt2f.toList();
        // # initialize a list of coordinates that will be ordered
        // # such that the first entry in the list is the top-left,
        // # the second entry is the top-right, the third is the
        // # bottom-right, and the fourth is the bottom-left
        java.util.Collections.sort(points, new java.util.Comparator<Point>() {
            @Override
            public int compare(Point p1, Point p2) {
                double s1 = p1.x + p1.y;
                double s2 = p2.x + p2.y;
                return Double.compare(s1, s2);
            }
        });
        Point topLeft = points.get(0);
        Point bottomRight = points.get(3);


        // # now, compute the difference between the points, the
        // # top-right point will have the smallest difference,
        // # whereas the bottom-left will have the largest difference
        java.util.Collections.sort(points, new java.util.Comparator<Point>() {
            @Override
            public int compare(Point p1, Point p2) {
                double s1 = p1.y - p1.x  ;
                double s2 = p2.y - p2.x;
                return Double.compare(s1, s2);
            }
        });
        Point topRight = points.get(0);
        Point bottomLeft = points.get(3);

        Point[] pts = new Point[]{topLeft,topRight, bottomRight, bottomLeft};

        screenCnt2f = new MatOfPoint2f(pts);
        // System.out.println(screenCnt2f.dump());
        return screenCnt2f;
    }

//    @SuppressLint("MissingPermission")
//    public double[] findPoints(String imagePath, int frameWidth, int frameHeight){
//        byte[] byteArray = new byte[0];
//        final int pointSize = 8;
//        double[] points = new double[pointSize];
//        try{
//            Mat src = Imgcodecs.imread(imagePath);
//            Imgproc.resize(src, src, new Size(500, 500));
//            int imageHeight = src.rows();
//            int imageWidth = src.cols();
//            double X_factor = (double)imageWidth/(double)frameWidth;
//            double Y_factor = (double)imageHeight/(double)frameHeight;
//            double minArea = (double)imageWidth * (double)imageHeight * .16;
//
//            Mat srcGray = new Mat();
//            Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);
//            Core.normalize(srcGray, srcGray, 0, 255d, Core.NORM_MINMAX);
//            // Thresholding
//            System.out.println("Thresholding");
//            Imgproc.threshold(srcGray, srcGray, 190, 255, Imgproc.THRESH_TRUNC);
//            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10d, 10d));
//            Imgproc.morphologyEx(srcGray, srcGray, Imgproc.MORPH_CLOSE, kernel);
//            // Edge
//            System.out.println("Edge");
//            Mat edge = new Mat();
//            Imgproc.Canny(srcGray, edge, 185, 55);
//            // Contours
//            System.out.println("Contours");
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(edge, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//            System.out.println("Contours " + contours.size());
//            // Filtering
//            System.out.println("Filtering");
//            List<MatOfPoint> filtered = new ArrayList<>();
//            for (int i = 0; i < contours.size(); i++) {
//                MatOfInt hull = new MatOfInt();
//                MatOfPoint contour = contours.get(i);
//                Imgproc.convexHull(contour, hull);
//                MatOfPoint mopHull = new MatOfPoint();
//                mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
//                for (int j = 0; j < hull.size().height; j++) {
//                    int index = (int) hull.get(j, 0)[0];
//                    double[] point = new double[]{contour.get(index, 0)[0], contour.get(index, 0)[1]};
//                    mopHull.put(j, 0, point);
//                }
//                double area = Imgproc.contourArea(mopHull);
//                if (area < minArea)
//                    continue;
//                filtered.add(mopHull);
//            }
//            // Choose Max
//            System.out.println("Choose max " + filtered.size());
//            double maxVal = 0;
//            int maxValIdx = -1;
//            MatOfPoint2f maxContour = new MatOfPoint2f();
//            for (int contourIdx = 0; contourIdx < filtered.size(); contourIdx++)
//            {
//                MatOfPoint contour = filtered.get(contourIdx);
//                double contourArea = Imgproc.contourArea(contour);
//                //System.out.println("contourArea " + contourArea);
//                if (maxVal < contourArea)
//                {
//                    double peri = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
//                    //System.out.println("peri " + peri);
//                    MatOfPoint2f approx = new MatOfPoint2f();
//                    Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()),approx,peri*0.025,true);
//                    //check its rectangle-ness:
//                    System.out.println(approx.size(0));
//                    //System.out.println(approx.toString());
//                    if(approx.size(0) == 4) {
//                        maxVal = contourArea;
//                        maxValIdx = contourIdx;
//                        maxContour = approx;
//                    }
//                }
//            }
//            //output.get(maxValIdx).toArray();
//            if(maxValIdx != -1) {
//                maxContour = orderPointsClockwise(maxContour);
//                System.out.println("OpenCV Result " + maxContour.toArray());
//                points[0] = maxContour.toArray()[0].x/X_factor; points[1] = maxContour.toArray()[0].y/Y_factor;
//                points[2] = maxContour.toArray()[1].x/X_factor; points[3] = maxContour.toArray()[1].y/Y_factor;
//                points[4] = maxContour.toArray()[2].x/X_factor; points[5] = maxContour.toArray()[2].y/Y_factor;
//                points[6] = maxContour.toArray()[3].x/X_factor; points[7] = maxContour.toArray()[3].y/Y_factor;
//            }else{
//                points[0] = 0; points[1] = 0;
//                points[2] = 0; points[3] = 0;
//                points[4] = 0; points[5] = 0;
//                points[6] = 0; points[7] = 0;
//            }
//
//
//
//            System.out.println("OpenCV findPoints " + points.toString());
//        }catch (Exception e){
//            System.out.println("OpenCV Error: " + e.toString());
//        }
//        return points;
//    }

    @SuppressLint("MissingPermission")
    public double[] findPoints(String imagePath, int frameWidth, int frameHeight){
        byte[] byteArray = new byte[0];
        final int pointSize = 8;
        double[] points = new double[pointSize];
        try{
            Mat src = Imgcodecs.imread(imagePath);
            Imgproc.resize(src, src, new Size(500, 500));

            int imageHeight = src.rows();
            int imageWidth = src.cols();
            double X_factor = (double)imageWidth/(double)frameWidth;
            double Y_factor = (double)imageHeight/(double)frameHeight;
            double minArea = (double)imageWidth * (double)imageHeight * .16;

            Mat srcGray = new Mat();
            Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.medianBlur(srcGray,srcGray,5);
            Core.normalize(srcGray, srcGray, 0, 255d, Core.NORM_MINMAX);
            // Thresholding
            System.out.println("Thresholding");
            Imgproc.threshold(srcGray, srcGray, 190, 255, Imgproc.THRESH_TRUNC);
            Core.normalize(srcGray, srcGray, 0, 255d, Core.NORM_MINMAX);
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15d, 15d));
            Imgproc.morphologyEx(srcGray, srcGray, Imgproc.MORPH_CLOSE, kernel);
            // Edge
            System.out.println("Edge");
            Mat edge = new Mat();
            Imgproc.Canny(srcGray, edge, 185, 55);
            // Contours
            System.out.println("Contours");
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(edge, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            System.out.println("Contours " + contours.size());
            // Filtering
            System.out.println("Filtering");
            List<MatOfPoint> filtered = new ArrayList<>();
            for (int i = 0; i < contours.size(); i++) {
                MatOfInt hull = new MatOfInt();
                MatOfPoint contour = contours.get(i);
                Imgproc.convexHull(contour, hull);
                MatOfPoint mopHull = new MatOfPoint();
                mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
                for (int j = 0; j < hull.size().height; j++) {
                    int index = (int) hull.get(j, 0)[0];
                    double[] point = new double[]{contour.get(index, 0)[0], contour.get(index, 0)[1]};
                    mopHull.put(j, 0, point);
                }
                double area = Imgproc.contourArea(mopHull);
                if (area < minArea)
                    continue;
                filtered.add(mopHull);
            }
            // Choose Max
            System.out.println("Choose max " + filtered.size());
            double maxVal = 0;
            int maxValIdx = -1;
            MatOfPoint2f maxContour = new MatOfPoint2f();
            for (int contourIdx = 0; contourIdx < filtered.size(); contourIdx++)
            {
                MatOfPoint contour = filtered.get(contourIdx);
                double contourArea = Imgproc.contourArea(contour);
                //System.out.println("contourArea " + contourArea);
                if (maxVal < contourArea)
                {
                    double peri = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
                    //System.out.println("peri " + peri);
                    MatOfPoint2f approx = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()),approx,peri*0.025,true);
                    //check its rectangle-ness:
                    System.out.println(approx.size(0));
                    //System.out.println(approx.toString());
                    if(approx.size(0) == 4) {
                        maxVal = contourArea;
                        maxValIdx = contourIdx;
                        maxContour = approx;
                    }
                }
            }
            //output.get(maxValIdx).toArray();
            if(maxValIdx != -1) {
                maxContour = orderPointsClockwise(maxContour);
                System.out.println("OpenCV Result " + maxContour.toArray());
                points[0] = maxContour.toArray()[0].x/X_factor; points[1] = maxContour.toArray()[0].y/Y_factor;
                points[2] = maxContour.toArray()[1].x/X_factor; points[3] = maxContour.toArray()[1].y/Y_factor;
                points[4] = maxContour.toArray()[2].x/X_factor; points[5] = maxContour.toArray()[2].y/Y_factor;
                points[6] = maxContour.toArray()[3].x/X_factor; points[7] = maxContour.toArray()[3].y/Y_factor;
            }else{
                points[0] = 0; points[1] = 0;
                points[2] = 0; points[3] = 0;
                points[4] = 0; points[5] = 0;
                points[6] = 0; points[7] = 0;
            }



            System.out.println("OpenCV findPoints " + points.toString());
        }catch (Exception e){
            System.out.println("OpenCV Error: " + e.toString());
        }
        return points;
    }

    @SuppressLint("MissingPermission")
    public byte[] document(byte[] byteData) {
        float clipHistPercent= 0;//.15f;
        byte[] byteArray = new byte[0];

        try {
            byte[] thresholdedByteData = photoCopy(byteData);
            //thresholdedByteData = adaptiveThreshold(thresholdedByteData,255,
            //        Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, 0, 11, 10);

            //byte[] thresholdedByteData = threshold(byteData,190, 255, 0);

            byte[] normalisedByteData  = autoNormalise(byteData, 0.01f);
            // Decode image from input byte array
            Mat dst = Imgcodecs.imdecode(new MatOfByte(normalisedByteData), Imgcodecs.IMREAD_UNCHANGED);

            // B/W mask on top
            Mat bw = Imgcodecs.imdecode(new MatOfByte(thresholdedByteData), Imgcodecs.IMREAD_UNCHANGED);
            Imgproc.cvtColor(bw, bw, Imgproc.COLOR_GRAY2BGR);
            double param = 0.3;
            Core.addWeighted(bw, param, dst, 1-param, 0, dst);
            System.out.println("AutoEnhance completed");

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }
    @SuppressLint("MissingPermission")
    public byte[] photoCopy(byte[] byteData) {
        byte[] byteArray = new byte[0];

        try {
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);
            Mat dst = new Mat();
            Mat bg = new Mat();
            Mat kernel = Mat.ones(7,7, CV_8UC1);
            Imgproc.dilate(src,bg,kernel);
            Imgproc.medianBlur(bg,bg,21);
            Core.absdiff(src, bg, bg);
            Imgproc.cvtColor(bg, dst, Imgproc.COLOR_BGR2GRAY);
            //Core.subtract(Scalar.all(255), dst, dst);
            Mat invertcolormatrix= new Mat(dst.rows(),dst.cols(), dst.type(), new Scalar(255,255,255));
            Core.subtract(invertcolormatrix, dst, dst);
            //dst = Scalar.all(255) - dst;
            Core.normalize(dst, dst, 0, 255d, Core.NORM_MINMAX);
            //Imgproc.threshold(dst, dst, 190, 255, 0);
            Mat dst1 = new Mat();
            Imgproc.adaptiveThreshold(dst, dst1, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, 0, 11, 10);
            Core.addWeighted(dst1, 0.3, dst, 0.7, 0, dst);

            System.out.println("Remove Shadow");
            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] autoNormalise(byte[] byteData, float clipHistPercent) {
        //float clipHistPercent= 0;//.15f;
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_UNCHANGED);
            Size size = new Size((double) 3d, 3d);
            // Convert the image to Gray
            //Imgproc.GaussianBlur(src, src, size, 0);

            int histSize = 256;
            double alpha, beta;
            double minGray = 0, maxGray = 0;
            System.out.println(CvType.CV_8UC1);
            System.out.println(CvType.CV_8UC3);
            System.out.println(CvType.CV_8UC4);

            System.out.println(src.type());

            //to calculate grayscale histogram
            Mat gray = new Mat();
            if (src.type() == CvType.CV_8UC1) gray = src;
            if (src.type() == CvType.CV_8UC3) Imgproc.cvtColor(src, gray, Imgproc.COLOR_BGR2GRAY);
            if (clipHistPercent == 0)
            {
                // keep full available range
                Core.MinMaxLocResult d = Core.minMaxLoc(gray);
                minGray = d.minVal;
                maxGray = d.maxVal;
            }else{
                Mat hist = new Mat(); //the grayscale histogram
                Mat mask = new Mat();
                MatOfFloat histRange = new MatOfFloat(0f, 256f);
                MatOfInt histSize_ = new MatOfInt(256);
                MatOfInt channels = new MatOfInt(0);
                ArrayList<Mat> listOfMat = new ArrayList<>();
                listOfMat.add(gray);
                boolean accumulate = false;
                Imgproc.calcHist(listOfMat, channels, mask, hist, histSize_, histRange, accumulate);

                double accumulator[];
                accumulator = new double[histSize];
                accumulator[0] = hist.get(0,0)[0];
                for (int i = 1; i < histSize; i++){
                    accumulator[i] = accumulator[i - 1] + hist.get(i,0)[0];
                }
                // locate points that cuts at required value
                double max = accumulator[histSize-1];
                clipHistPercent *= (max / 100.0); //make percent as absolute
                clipHistPercent /= 2.0; // left and right wings
                // locate left cut
                minGray = 0;
                while (accumulator[(int)minGray] < clipHistPercent)
                    minGray++;

                // locate right cut
                maxGray = histSize - 1;
                while (accumulator[(int)maxGray] >= (max - clipHistPercent))
                    maxGray--;
            }
            // current range
            //double inputRange = maxGray - minGray;
            double inputRange = maxGray - minGray;
            alpha = (histSize - 1) / inputRange;   // alpha expands current range to histsize range
            beta = minGray * alpha;             // beta shifts current range so that minGray will go to 0
            //alpha = 3.072289156626506;
            //beta = -144.3975903614458;

            // Apply brightness and contrast normalization
            // convertTo operates with saurate_cast
            src.convertTo(dst, -1, alpha, beta);

            System.out.println("autoNormalise completed");

            //instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }





}
