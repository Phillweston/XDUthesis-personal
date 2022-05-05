private void kalmanInit() {

    kf = new KalmanFilter(2, 2, 0, CV_32F);

    Log.d(TAG, "CHECKING KalmanFilter Matrix: " +
            "\n transitionMatrix: " + kf.get_transitionMatrix().size() +
            "\n measurementMatrix: " + kf.get_measurementMatrix().size() +
            "\n statePre: " + kf.get_statePre().size() +
            "\n processionNoiseCov: " + kf.get_processNoiseCov().size() +
            "\n measurementNoiseCov: " + kf.get_measurementNoiseCov().size() +
            "\n errorCovPost: " + kf.get_errorCovPost().size());

    kalmanMatrixInit2();
}

private void kalmanMatrixInit4() {
//        CHECKING KalmanFilter Matrix:
//     transitionMatrix: 4x4
//     measurementMatrix: 4x2
//     statePre: 1x4
//     processionNoiseCov: 4x4
//     measurementNoiseCov: 2x2
//     errorCovPost: 4x4
    transitionMatrix = Mat.eye(4, 4, CV_32F);
    transitionMatrix.put(4, 4, new float[]{1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1});
    kf.set_transitionMatrix(transitionMatrix);

    measurementMatrix = Mat.eye(2, 4, CV_32F);
    kf.set_measurementMatrix(measurementMatrix);

    statePre = new Mat(4,1, CV_32F);
    statePre.put(0, 0, 0.5f);
    statePre.put( 1,0, 0.5f);
    statePre.put( 2, 0,0);
    statePre.put( 3,0, 0);
    kf.set_statePre(statePre);

    processNoiseCov = Mat.eye(4, 4, CV_32F);
    processNoiseCov = processNoiseCov.mul(processNoiseCov, 1e-1);
    kf.set_processNoiseCov(processNoiseCov);

    measurementNoiseCov = Mat.eye(2, 2, CV_32F);
    measurementNoiseCov = measurementNoiseCov.mul(measurementNoiseCov, 1e-1);
    kf.set_measurementNoiseCov(measurementNoiseCov);

    errorCovPost = Mat.eye(4, 4, CV_32F);
    errorCovPost = errorCovPost.mul(errorCovPost, 0.1);
    kf.set_errorCovPost(errorCovPost);
}

private void kalmanMatrixInit2() {
    //        CHECKING KalmanFilter Matrix:
//        transitionMatrix: 2x2
//        measurementMatrix: 2x2
//        statePre: 1x2
//        processionNoiseCov: 2x2
//        measurementNoiseCov: 2x2
//        errorCovPost: 2x2
    transitionMatrix = Mat.ones(2, 2, CV_32F);
    kf.set_transitionMatrix(transitionMatrix);

    measurementMatrix = Mat.eye(2, 2, CV_32F);
    kf.set_measurementMatrix(measurementMatrix);

    statePre = new Mat(2,1, CV_32F);
    statePre.put(0, 0, 0.5f);
    statePre.put( 1,0, 0.5f);
    kf.set_statePre(statePre);

    processNoiseCov = Mat.eye(2, 2, CV_32F);
    processNoiseCov = processNoiseCov.mul(processNoiseCov, 1e-1);
    kf.set_processNoiseCov(processNoiseCov);

    measurementNoiseCov = Mat.eye(2, 2, CV_32F);
    measurementNoiseCov = measurementNoiseCov.mul(measurementNoiseCov, 1e-1);
    kf.set_measurementNoiseCov(measurementNoiseCov);

    errorCovPost = Mat.eye(2, 2, CV_32F);
    errorCovPost = errorCovPost.mul(errorCovPost, 0.1);
    kf.set_errorCovPost(errorCovPost);
}

private void kalmanWork() {
    //kalman

    //update
    kf.get_measurementMatrix().put(0, 0, targetPoint.x*videoWidth);
    kf.get_measurementMatrix().put(1, 0, targetPoint.y*videoWidth);

    Mat measure = new Mat(2, 1, CV_32F);
    measure.put(0, 0, targetPoint.x*videoWidth);
    measure.put(1, 0, targetPoint.y*videoHeight);

    //correct
    Mat test = kf.correct(measure);
    Log.d(TAG, "correct: " + test.size());

    //predict
    Mat prediction = kf.predict();
    if(prediction != null) {
        Log.d(TAG, "prediction size: " + prediction.size());
//            Log.d(TAG, "prediction1: " + prediction.get(0, 0)[0]);
//            Log.d(TAG, "prediction2: " + prediction.get(0, 0)[1]);
//            Log.d(TAG, "prediction3: " + prediction.get(1, 0)[0]);
    }else{
        Log.d(TAG, "predictionIsNull");
    }
    if(prediction.get(0,0) != null && prediction.get(1,0) != null) {
        PointF predictPt = new PointF((float) prediction.get(0, 0)[0], (float) prediction.get(1, 0)[0]);
        targetPoint = predictPt;
        targetPoint.x = targetPoint.x / videoWidth;
        targetPoint.y = targetPoint.y / videoHeight;

        Log.d(TAG, "prediction point: " + predictPt);
    }
}

public boolean isTargetInVision() {
    targetPoint = getTargetPoint(testingActivity.yuv);
    if(targetPoint != null) {
        return true;
    }
    return false;
}