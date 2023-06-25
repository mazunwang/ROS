/**
 * @file parameters_config.hpp
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef PARAMETERS_CONFIG_HPP_
#define PARAMETERS_CONFIG_HPP_

#include <iostream>
#include <string>
#include <unordered_map>
#include <sstream>
#include <type_traits>


namespace parameter{

template<typename T>
struct Parameter{
    std::string type;
    T value;
    std::string description;
    T min_value;
    T max_value;
};

enum ParameterType{
    kOther = -1,
    kInt = 0,
    kDouble,
    kBool,
};

class ParametersConfig {
public:
    ParametersConfig(){
        double_parameters_map_.clear();
        int_parameters_map_.clear();
        bool_parameters_map_.clear();
    }
    ~ParametersConfig(){

    }

    template<typename T>
    bool SetParameter(const std::string name, T value){
        auto it = double_parameters_map_.find(name);
        if (it != double_parameters_map_.end()){
            it->second.value = GetLimitValue<T>(value, it->second.min_value, it->second.max_value);
        }else{
            
        }
    }

    template<typename T>
    T GetParameter(const std::string& name){
        switch (GetParameterType<T>()){
            case kInt:
                return GetIntParameter(name);
                break;
            case kDouble:
                return GetDoubleParameter(name);
                break;
            case kBool:
                return GetBoolParameter(name);
                break;
            
            default:
                break;
        }
        return T();
    }

    template<typename T>
    void AddParameter(const std::string& name, const T& value, const std::string& description, const T& min_value, const T& max_value){
        switch (GetParameterType<T>())
        {
        case kInt:{
            // std::cout << "int" << std::endl;
            Parameter<int> para;
            para.description = description;
            para.min_value = min_value;
            para.max_value = max_value;
            para.value = GetLimitValue<int>(value, min_value, max_value);
            int_parameters_map_[name] = para;
        }
            break;
        case kDouble:{
            // std::cout << "double" << std::endl;
            Parameter<double> para;
            para.value = value;
            para.description = description;
            para.min_value = min_value;
            para.max_value = max_value;
            para.value = GetLimitValue<double>(value, min_value, max_value);
            double_parameters_map_[name] = para;
        }
            break;

        case kBool:{
            // std::cout << "bool" << std::endl;
            Parameter<bool> para;
            para.value = value;
            para.description = description;
            para.min_value = false;
            para.max_value = true;
            bool_parameters_map_[name] = para;
        }
            break;
        
        default:
            break;
        }
    }

private:
    std::unordered_map<std::string, Parameter<double> > double_parameters_map_;
    std::unordered_map<std::string, Parameter<int> > int_parameters_map_;
    std::unordered_map<std::string, Parameter<bool> > bool_parameters_map_;

    template <typename T>
    T GetLimitValue(T value, const T& min_value, const T& max_value){
        if(GetParameterType<T>() == kBool) return value;
        if(min_value > max_value){

        }
        T res = value;
        res = std::min(std::max(min_value, value), max_value);
        return res;
    }

    

    template <typename T>
    ParameterType GetParameterType(){
        if (std::is_same<T, int>::value) {
            return ParameterType::kInt;
        } else if (std::is_same<T, double>::value) {
            return ParameterType::kDouble;
        } else if (std::is_same<T, bool>::value) {
            return ParameterType::kBool;
        } else {
            return ParameterType::kOther;
        }   
    }

    double GetDoubleParameter(const std::string& name){
        auto it = double_parameters_map_.find(name);
        if (it != double_parameters_map_.end()) {
            return it->second.value;
        }else {

        }
        return double();   
    }

    int GetIntParameter(const std::string& name){
        auto it = int_parameters_map_.find(name);
        if (it != int_parameters_map_.end()) {
            return it->second.value;
        }else {
            
        }
        return int();   
    }

    bool GetBoolParameter(const std::string& name){
        auto it = bool_parameters_map_.find(name);
        if (it != bool_parameters_map_.end()) {
            return it->second.value;
        }else {
            
        }
        return bool();   
    }
};

};

#endif