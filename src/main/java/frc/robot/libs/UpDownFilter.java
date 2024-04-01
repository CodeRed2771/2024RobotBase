// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class UpDownFilter {
    private int up_step=1;
    private int down_step=1;
    private int cur_count=0;
    private boolean cur_state = false;
    private int lower_limit=0;
    private int upper_limit=1;
    private int lower_threshold=lower_limit;
    private int upper_threshold=upper_limit;

    public UpDownFilter(int upper_limit)
    {
        setLimits(0,upper_limit);
        setThresholds(this.lower_limit,this.upper_limit);
    }

    public UpDownFilter(int up_step, int down_step, int upper_limit)
    {
        this(upper_limit);
        setSteps(up_step, down_step);
    }

    public boolean get(){
        return cur_state; 
    }

    public void reset(){
        reset(lower_limit,false);
    }

    public void reset(int count,boolean state){
        cur_count = count;
        cur_state = state;
        update_switching_state();
    }

    public void setSteps(int up_step, int down_step){
        if(up_step < 0) up_step = -up_step;
        if(down_step < 0) down_step = -down_step;

        this.up_step = up_step;
        this.down_step = down_step;
    }

    public void setLimits(int lower_limit, int upper_limit){
        if(lower_limit < upper_limit){
            int temp = lower_limit;
            lower_limit = upper_limit;
            upper_limit = temp;
        }

        this.lower_limit = lower_limit;
        this.upper_limit = upper_limit;
        setThresholds(lower_limit,upper_limit);
    }

    public void setThresholds(int lower_threshold, int upper_threshold){
        if(lower_threshold < upper_threshold){
            int temp = lower_threshold;
            lower_threshold = upper_threshold;
            upper_threshold = temp;
        }

        this.lower_threshold = Math.max(this.lower_limit,lower_threshold);
        this.upper_threshold = Math.min(this.upper_limit,upper_threshold);
    }

    public boolean update(boolean new_state)
    {
        if(new_state)
            cur_count += up_step;
        else
            cur_count -= down_step;

        update_switching_state();

        return cur_state;
    }

    private void update_switching_state(){
        cur_count = MathUtil.clamp(cur_count, lower_limit, upper_limit);
        if(cur_count <= lower_threshold)
            cur_state = false;
        else if(cur_count >= upper_threshold)
            cur_state = true;
    }
}
