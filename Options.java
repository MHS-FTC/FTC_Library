package org.firstinspires.ftc.teamcode.API;

import java.util.HashMap;

/**
 * Created by Ethan Hampton on 8/19/17.
 *
 * The configuration object for the subsystem that allows for multiple options to be set easily and effectively both by the subsystem and the robot using it.
 */

public class Options {
    private String name;

    private HashMap<String, String> options = new HashMap<>();

    public Options(String name){
        this.name = name;
    }

    public HashMap<String, String> getOptions() {
        return options;
    }

    public void set(HashMap<String, String> options) {
        this.options = options;
    }

    //add an option to the configuration
    public HashMap<String,String> add(String key, String value){
        options.put(key, value);
        return options;
    }

    //update an option to the configuration
    public HashMap<String,String> update(String key, String value){
        options.remove(key);
        options.put(key, value);
        return options;
    }

    //remove an option to the configuration
    public HashMap<String,String> remove(String key){
        options.remove(key);
        return options;
    }

    //get one option from the options hashmap
    public String get(String key){
        return options.get(key);
    }

    //gets name of module, subsystem or other system using this configuration
    public String getName() {
        return name;
    }

}
