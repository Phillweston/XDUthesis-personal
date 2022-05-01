//-------------------------Flight Control-------------------------
    @Subscribe(threadMode = ThreadMode.ASYNC)
    public void flightControl(TargetPointResultEvent targetPointResultEvent) {
        //receive the target detection result
        targetPoint = targetPointResultEvent.targetPoint;
        setFlightControlData(targetPoint);
    }


    float errorXI = 0;
    float errorYI = 0;
    float altitudeI = 0;
    float errorXPre = 0;
    float errorYPre = 0;
    float altitudePre = 0;
    public void setFlightControlData(PointF targetPoint) {

        float errorXCur = targetPoint.x - 0.5f;
        float errorYCur = targetPoint.y - 0.5f;
        errorXI += errorXCur;
        errorYI += errorYCur;

        //--------------------pitch--------------------
        //specific
        float pitchP = -0.0001f;
        float pitchI = 0f;
        float pitchD = -0.01f;
        //pitch angle: (0.00001, 0.001)
        float pitchAngle = pitchP*errorXCur+pitchI*errorXI+pitchD*(errorXCur-errorXPre);

        //--------------------roll--------------------
        //specific
        float rollP = 0.1f;
        float rollI = 0.001f;
        float rollD = 0.1f;
        //roll angle range in (-0.4, 0.4)
        float rollAngle = rollP*errorXCur+rollI*errorXI+rollD*(errorXCur-errorXPre);

        //--------------------vertical--------------------
        //specific
        float verticalThrottleP = -0.1f;
        float verticalThrottleI = -0.0001f;
        float verticalThrottleD = 0.1f;
        float altitudeCur = ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getAltitude();
        altitudeI += altitudeCur;
        //roll angle range in (-5, 5)
//        float verticalThrottle = verticalThrottleP*altitudeCur+verticalThrottleI*altitudeI+verticalThrottleD*(altitudeCur-altitudePre);
        float verticalThrottle = 0;

        flightControlData = new FlightControlData(pitchAngle, rollAngle, 0, verticalThrottle);

        errorXPre = errorXCur;
        errorYPre = errorYCur;
        altitudePre = altitudeCur;
    }

    public void setFlightControlData2() {
        //for test

        //build the center point
        float latitudeCur = (float) ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getLatitude();

        float longitudeCur = (float) ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getLongitude();

        float altitudeCur = ((Aircraft)RDApplication.getProductInstance()).getFlightController()
                .getState().getAircraftLocation().getAltitude();

        Point3D flightPoint = new Point3D(latitudeCur, longitudeCur, altitudeCur);

        float errorXCur = flightPoint.x - target.x;     //0.1
        float errorYCur = flightPoint.y - target.y;
        errorXI += errorXCur;
        errorYI += errorYCur;

        //--------------------pitch--------------------
        //specific
        float pitchP = -0.01f;
        float pitchI = 0.0000001f;
        float pitchD = 0.001f;
        //pitch angle: (0.00001, 0.001)
        float pitchAngle = pitchP*errorXCur+pitchI*errorXI+pitchD*(errorXCur-errorXPre);

        //--------------------roll--------------------
        //specific
        float rollP = 0.4f;
        float rollI = 0.0000001f;
        float rollD = 1f;
        //roll angle range in (-0.4, 0.4)
        float rollAngle = rollP*errorXCur+rollI*errorXI+rollD*(errorXCur-errorXPre);

        //--------------------vertical--------------------
        //specific
        float verticalThrottleP = -0.1f;
        float verticalThrottleI = 0;
        float verticalThrottleD = -0.1f;

        altitudeI += altitudeCur;
        //roll angle range in (-5, 5)
        float verticalThrottle = verticalThrottleP*altitudeCur+verticalThrottleI*altitudeI+verticalThrottleD*(altitudeCur-altitudePre);

        flightControlData = new FlightControlData(pitchAngle, rollAngle, 0, verticalThrottle);

        Log.d(TAG, "Flight control data: " + flightControlData.getPitch()+" "+flightControlData.getRoll()+" "+flightControlData.getVerticalThrottle());
        Log.d(TAG, "Flight body data: " + flightPoint.x+" "+flightPoint.y+" "+flightPoint.z);

        errorXPre = errorXCur;
        errorYPre = errorYCur;
        altitudePre = altitudeCur;
    }
