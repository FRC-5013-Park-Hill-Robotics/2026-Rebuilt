package frc.robot.trobot5013lib;

import java.util.ArrayDeque;
import java.util.Iterator;

public class AverageOverTime {
    public double mTimespace;
    public ArrayDeque<Double> mMessurements = new ArrayDeque<Double>();
    public ArrayDeque<Double> mTimestamps = new ArrayDeque<Double>();

    public double mOutlier;

    public AverageOverTime(double timespace){
        mTimespace = timespace;
        mOutlier = 999999999;
    }

    public AverageOverTime(double timespace, double outlier){
        mTimespace = timespace;
        mOutlier = outlier;
    }

    public void addMessurement(double messurement, double timestamp){
        if(messurement < mOutlier){
            mMessurements.push(messurement);
            mTimestamps.push(timestamp);
        }
    }

    public double getAverage(double currentTime){
        Iterator<Double> M = mMessurements.descendingIterator();
        Iterator<Double> T = mTimestamps.descendingIterator();
        double i = 0;
        double sum = 0;
        while(M.hasNext() && T.hasNext()){
            double m = M.next();
            double t = T.next();
            if(currentTime - t < mTimespace){
                sum += m;
                i++;
            }
        }
        //mMessurements.peekLast()
        double avg = sum/i;
        return avg;
    }

    public void clearHistory(){
        mMessurements.clear();
        mTimestamps.clear();
    }
}
