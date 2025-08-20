/*
 *  Copyright (c) 2024 Aplux Intelligence Technologies, Ltd.
 *  All rights reserved.
 *
 *  Confidential and Proprietary - Aplux Intelligence Technologies, Ltd.
 */

#include <thread>
#include <future>

#include <opencv2/opencv.hpp>

#include "aidlux/aidlite/aidlite.hpp"

using namespace Aidlux::Aidlite;
using namespace std;

int acc_type = 0;

#define OBJ_CLASS_NUM     80
#define NMS_THRESH        0.45
#define BOX_THRESH        0.5
#define MODEL_SIZE 640

#define OBJ_NUMB_MAX_SIZE 64
#define PROP_BOX_SIZE     (5+OBJ_CLASS_NUM)
#define STRIDE8_SIZE (MODEL_SIZE / 8)
#define STRIDE16_SIZE (MODEL_SIZE / 16)
#define STRIDE32_SIZE (MODEL_SIZE / 32)


const float anchor0[6] = {10, 13, 16, 30, 33, 23};
const float anchor1[6] = {30, 61, 62, 45, 59, 119};
const float anchor2[6] = {116, 90, 156, 198, 373, 326};

static float sigmoid(float x) { return 1.f / (1.f + exp(-x)); }

float eqprocess(cv::Mat* src, cv::Mat* dst, int width, int height)
{
    int w =  src->cols;
    int h =  src->rows;
    float scale_h = float(h) / float(height);
    float scale_w = float(w) / float(width);

    float scale;
    if (scale_h > scale_w)
    {
      scale = scale_h;
    }
    else
    {
      scale = scale_w;
    }

    int rel_width = int(w / scale);
    int rel_height = int(h / scale);

    cv::Mat tmp = (*dst)(cv::Rect(0, 0, rel_width, rel_height));
    cv::resize(*src, tmp, cv::Size(rel_width, rel_height));
    return scale;
}

std::vector<std::string> split(const std::string& str)
{
    std::stringstream ss(str);
    std::vector<std::string> elems;
    std::string item;
    while (std::getline(ss, item, ','))
    {
        elems.push_back(item);
    }
    return elems;
}


int process(float* output, std::vector<float>& boxes, std::vector<float>& objProbs, std::vector<int>& classId, float * anchor, int grid_h, int grid_w, int stride, int imgsz)
{
    int ct = 0;
    int validCount = 0;
    for (int a = 0; a < 3; a++)
    {
        for (int i = 0; i < grid_h; i++)
        {
            for (int j = 0; j < grid_w; j++)
            {
                int idx = a * PROP_BOX_SIZE + (i * grid_w + j) * 3 * PROP_BOX_SIZE;
                float box_confidence = sigmoid(output[idx + 4]);
                if (box_confidence >= BOX_THRESH )
                {
                    float box_x  = sigmoid(output[idx]) * 2 - 0.5;
                    float box_y  = sigmoid(output[idx + 1]) * 2 - 0.5;
                    float box_w  = pow(sigmoid(output[idx + 2]) * 2, 2);
                    float box_h  = pow(sigmoid(output[idx + 3]) * 2, 2);

                    box_x = (box_x + j) * (float)stride;
                    box_y = (box_y + i) * (float)stride;
                    box_w = box_w * anchor[a * 2];
                    box_h = box_h * anchor[a * 2 + 1];

                    box_x -= (box_w / 2.0);
                    box_y -= (box_h / 2.0);

                    float maxClassProbs = 0;
                    int maxClassId = 0;

                    for(int k = 0; k < OBJ_CLASS_NUM ; k++)
                    {
                        float prob = output[idx + 5 + k];
                        if (prob > maxClassProbs)
                        {
                            maxClassId = k;
                            maxClassProbs = prob;
                        }
                    }
                    if (maxClassProbs > BOX_THRESH)
                    {
                        objProbs.push_back(sigmoid(maxClassProbs) * box_confidence);
                        classId.push_back(maxClassId);
                        validCount++;
                        boxes.push_back(box_x);
                        boxes.push_back(box_y);
                        boxes.push_back(box_w);
                        boxes.push_back(box_h);
                    }
                }
            }
        }
    }
      
    return validCount;
}


static int quick_sort_indice_inverse(std::vector<float>& input, int left, int right, std::vector<int>& indices)
{
    float key;
    int   key_index;
    int   low  = left;
    int   high = right;
    if (left < right) {
    key_index = indices[left];
    key       = input[left];
    while (low < high) {
        while (low < high && input[high] <= key) {
        high--;
        }
        input[low]   = input[high];
        indices[low] = indices[high];
        while (low < high && input[low] >= key) {
        low++;
        }
        input[high]   = input[low];
        indices[high] = indices[low];
    }
    input[low]   = key;
    indices[low] = key_index;
    quick_sort_indice_inverse(input, left, low - 1, indices);
    quick_sort_indice_inverse(input, low + 1, right, indices);
    }
    return low;
}

static float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1,
                              float ymax1)
{
  float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
  float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
  float i = w * h;
  float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) + (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
  return u <= 0.f ? 0.f : (i / u);
}


static int nms(int validCount, std::vector<float>& outputLocations, std::vector<int> classIds, std::vector<int>& order,
               int filterId, float threshold)
{
  for (int i = 0; i < validCount; ++i) {
    if (order[i] == -1 || classIds[i] != filterId) {
      continue;
    }
    int n = order[i];
    for (int j = i + 1; j < validCount; ++j) {
      int m = order[j];
      if (m == -1 || classIds[i] != filterId) {
        continue;
      }
      float xmin0 = outputLocations[n * 4 + 0];
      float ymin0 = outputLocations[n * 4 + 1];
      float xmax0 = outputLocations[n * 4 + 0] + outputLocations[n * 4 + 2];
      float ymax0 = outputLocations[n * 4 + 1] + outputLocations[n * 4 + 3];

      float xmin1 = outputLocations[m * 4 + 0];
      float ymin1 = outputLocations[m * 4 + 1];
      float xmax1 = outputLocations[m * 4 + 0] + outputLocations[m * 4 + 2];
      float ymax1 = outputLocations[m * 4 + 1] + outputLocations[m * 4 + 3];

      float iou = CalculateOverlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1);

      if (iou > threshold) {
        order[j] = -1;
      }
    }
  }
  return 0;
}

int32_t show_help_prompt(){

    printf("Executable Program parameter unexpected. Usage : \n");
    // printf("        [./qnn_yolov5_multi 1 ] Run the .so model on the CPU.\n");
    // printf("        [./qnn_yolov5_multi 2 ] Run the .so model on the GPU.\n");
    printf("        [./qnn_yolov5_multi 3 ] Run the .bin model on the DSP.\n");
    // printf("        [./qnn_yolov5_multi 4 ] Run the .so model on the DSP.\n");

    return EXIT_SUCCESS;
}

int32_t thread_func(int thread_idx){

    printf("entry thread_func[%d]\n", thread_idx);

    std::string image_path = "../../data/qnn_yolov5_multi/bus.jpg";
    std::string save_name = "out_yolov5_qnn";
    std::string model_path = "";

    if(acc_type == 1){
        model_path = "../../data/qnn_yolov5_multi/libcutoff_yolov5s_sigmoid_fp32.qnn229.aarch64.gcc9_4.so";
    }else if(acc_type == 2){
        model_path = "../../data/qnn_yolov5_multi/libcutoff_yolov5s_sigmoid_fp32.qnn229.aarch64.gcc9_4.so";
    }else if(acc_type == 3){
        model_path = "../../data/qnn_yolov5_multi/cutoff_yolov5s_sigmoid_w8a8.qnn229.ctx.bin";
        // model_path = "../../data/qnn_yolov5_multi/cutoff_yolov5s_sigmoid_w8a16.qnn229.ctx.bin";
        // model_path = "../../data/qnn_yolov5_multi/cutoff_yolov5s_sigmoid_fp16.qnn229.ctx.bin";
        // model_path = "../../data/qnn_yolov5_multi/cutoff_yolov5s_sigmoid_w8a8.qnn229.ctx.bin.aidem2";
    }else if(acc_type == 4){
        model_path = "../../data/qnn_yolov5_multi/libcutoff_yolov5s_sigmoid_w8a8.qnn229.aarch64.gcc9_4.so";
        // model_path = "../../data/qnn_yolov5_multi/libcutoff_yolov5s_sigmoid_w8a16.qnn229.aarch64.gcc9_4.so";
        // model_path = "../../data/qnn_yolov5_multi/libcutoff_yolov5s_sigmoid_fp16.qnn229.aarch64.gcc9_4.so";
        // model_path = "../../data/qnn_yolov5_multi/libcutoff_yolov5s_sigmoid_w8a8.qnn229.aarch64.gcc9_4.so.aidem2";
    }else{
        show_help_prompt();
        return EXIT_FAILURE;
    }

    // image process
    cv::Mat frame = cv::imread(image_path);
    cv::cvtColor(frame, frame , cv::COLOR_BGR2RGB);
    cv::Scalar stds_scale(255, 255, 255);
    cv::Size target_shape(MODEL_SIZE, MODEL_SIZE);

    cv::Mat frame_resized = cv::Mat::zeros(MODEL_SIZE, MODEL_SIZE, CV_8UC3);
    float scale = eqprocess(&frame, &frame_resized, MODEL_SIZE, MODEL_SIZE);

    cv::Mat input_data;
    frame_resized.convertTo(input_data, CV_32FC3);
    cv::divide(input_data, stds_scale, input_data);
    
    // model init

    printf("Aidlite library version : %s\n", Aidlux::Aidlite::get_library_version().c_str());

    // 以下三个接口请按需组合调用。如果不调用这些函数，默认只打印错误日志到标准错误终端。
    Aidlux::Aidlite::set_log_level(Aidlux::Aidlite::LogLevel::INFO);
    Aidlux::Aidlite::log_to_stderr();
    // Aidlux::Aidlite::log_to_file("./qnn_yolov5_multi_");

    Model* model = Model::create_instance(model_path);
    if(model == nullptr){
        printf("Create Model object failed !\n");
        return EXIT_FAILURE;
    }
    std::vector<std::vector<uint32_t>> input_shapes = {{1,640,640,3}};
    std::vector<std::vector<uint32_t>> output_shapes = {{1,40,40,255}, {1,20,20,255}, {1,80,80,255}};
    model->set_model_properties(input_shapes, DataType::TYPE_FLOAT32, output_shapes, DataType::TYPE_FLOAT32);

    Config* config = Config::create_instance();
    if(config == nullptr){
        printf("Create Config object failed !\n");
        return EXIT_FAILURE;
    }
    
    config->implement_type = ImplementType::TYPE_LOCAL;
    config->framework_type = FrameworkType::TYPE_QNN229;
    if(acc_type == 1){
        config->accelerate_type = AccelerateType::TYPE_CPU;
    }else if(acc_type == 2){
        config->accelerate_type = AccelerateType::TYPE_GPU;
    }else if(acc_type == 3){
        config->accelerate_type = AccelerateType::TYPE_DSP;
    }else if(acc_type == 4){
        config->accelerate_type = AccelerateType::TYPE_DSP;
    }else{
        show_help_prompt();
        return EXIT_FAILURE;
    }

    // std::vector<std::string> output_node_names = split(outNode_names);
    // config->snpe_out_names = output_node_names;
    std::unique_ptr<Interpreter>&& qnn_interpreter = InterpreterBuilder::build_interpretper_from_model_and_config(model, config);
    if(qnn_interpreter == nullptr){
        printf("build_interpretper_from_model_and_config failed !\n");
        return EXIT_FAILURE;
    }

    int result = qnn_interpreter->init();
    if(result != EXIT_SUCCESS){
        printf("interpreter->init() failed !\n");
        return EXIT_FAILURE;
    }
    
    result = qnn_interpreter->load_model();
    if(result != EXIT_SUCCESS){
        printf("interpreter->load_model() failed !\n");
        return EXIT_FAILURE;
    }

    printf("load model load success!\n");

    std::vector<std::vector<TensorInfo>> in_tensor_info;
    result = qnn_interpreter->get_input_tensor_info(in_tensor_info);
    if(result != EXIT_SUCCESS){
        printf("interpreter->get_input_tensor_info() failed !\n");
        return EXIT_FAILURE;
    }
    for(int gi = 0; gi < in_tensor_info.size(); ++gi){
       for(int ti = 0; ti < in_tensor_info[gi].size(); ++ti){

           std::string tensor_shape_str = std::to_string(in_tensor_info[gi][ti].shape[0]);
           for(int si = 1; si < in_tensor_info[gi][ti].dimensions; ++si){
               tensor_shape_str += " ";
               tensor_shape_str += std::to_string(in_tensor_info[gi][ti].shape[si]);
           }

           printf("Input  tensor : Graph[%d]-Tensor[%d]-name[%s]-element_count[%ld]-element_type[%d]-dimensions[%d]-shape[%s]\n",
                   gi, ti, in_tensor_info[gi][ti].name.c_str(), in_tensor_info[gi][ti].element_count, 
                   (int)in_tensor_info[gi][ti].element_type, in_tensor_info[gi][ti].dimensions, tensor_shape_str.c_str());
       }
    }

    std::vector<std::vector<TensorInfo>> out_tensor_info;
    result = qnn_interpreter->get_output_tensor_info(out_tensor_info);
    if(result != EXIT_SUCCESS){
        printf("interpreter->get_output_tensor_info() failed !\n");
        return EXIT_FAILURE;
    }
    for(int gi = 0; gi < out_tensor_info.size(); ++gi){
       for(int ti = 0; ti < out_tensor_info[gi].size(); ++ti){

           std::string tensor_shape_str = std::to_string(out_tensor_info[gi][ti].shape[0]);
           for(int si = 1; si < out_tensor_info[gi][ti].dimensions; ++si){
               tensor_shape_str += " ";
               tensor_shape_str += std::to_string(out_tensor_info[gi][ti].shape[si]);
           }

           printf("Output tensor : Graph[%d]-Tensor[%d]-name[%s]-element_count[%ld]-element_type[%d]-dimensions[%d]-shape[%s]\n",
                   gi, ti, out_tensor_info[gi][ti].name.c_str(), out_tensor_info[gi][ti].element_count, 
                   (int)out_tensor_info[gi][ti].element_type, out_tensor_info[gi][ti].dimensions, tensor_shape_str.c_str());
       }
    }

    float* stride8 = nullptr;
    float* stride16 = nullptr;
    float* stride32 = nullptr;

    // post_process
    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int>   classId;

    double sum_time_0 = 0.0, sum_time_1 = 0.0, sum_time_2 = 0.0;
    int _counter = 10;
    for(int idx = 0; idx < _counter; ++idx){
        std::chrono::steady_clock::time_point st0 = std::chrono::steady_clock::now();

        // if(acc_type == 0){
        //     std::this_thread::sleep_for(std::chrono::milliseconds(40));
        // }

        void* input_tensor_data = (void*)input_data.data;
        // result = qnn_interpreter->set_input_tensor(0, input_tensor_data);
        result = qnn_interpreter->set_input_tensor("images",input_tensor_data);
        if(result != EXIT_SUCCESS){
            printf("interpreter->set_input_tensor() failed !\n");
            return EXIT_FAILURE;
        }

        std::chrono::steady_clock::time_point et0 = std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration dur0 = et0 - st0;
        printf("current thread_idx[%d] [%d] set_input_tensor cost time : %f\n", thread_idx, idx, std::chrono::duration<double>(dur0).count()*1000);
        sum_time_0 += std::chrono::duration<double>(dur0).count()*1000;

        std::chrono::steady_clock::time_point st1 = std::chrono::steady_clock::now();

        result = qnn_interpreter->invoke();
        if(result != EXIT_SUCCESS){
            printf("interpreter->invoke() failed !\n");
            return EXIT_FAILURE;
        }

        std::chrono::steady_clock::time_point et1 = std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration dur1 = et1 - st1;
        printf("current thread_idx[%d] [%d] invoke cost time : %f\n", thread_idx, idx, std::chrono::duration<double>(dur1).count()*1000);
        sum_time_1 += std::chrono::duration<double>(dur1).count()*1000;
        
        std::chrono::steady_clock::time_point st2 = std::chrono::steady_clock::now();
        
        if(acc_type == 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        uint32_t output_tensor_length_0 = 0;
        // result = qnn_interpreter->get_output_tensor(0, (void**)&stride8, &output_tensor_length_0);
        result = qnn_interpreter->get_output_tensor("_326", (void**)&stride8, &output_tensor_length_0);
        if(result != EXIT_SUCCESS){
            printf("interpreter->get_output_tensor() 0 failed !\n");
            return EXIT_FAILURE;
        }
        printf("sample : interpreter->get_output_tensor() 0 length is [%d] !\n", output_tensor_length_0);

        uint32_t output_tensor_length_1 = 0;
        // result = qnn_interpreter->get_output_tensor(1, (void**)&stride16, &output_tensor_length_1);
        result = qnn_interpreter->get_output_tensor("_364", (void**)&stride16, &output_tensor_length_1);
        if(result != EXIT_SUCCESS){
            printf("interpreter->get_output_tensor() 1 failed !\n");
            return EXIT_FAILURE;
        }
        printf("sample : interpreter->get_output_tensor() 1 length is [%d] !\n", output_tensor_length_1);

        uint32_t output_tensor_length_2 = 0;
        // result = qnn_interpreter->get_output_tensor(2, (void**)&stride32, &output_tensor_length_2);
        result = qnn_interpreter->get_output_tensor("_402", (void**)&stride32, &output_tensor_length_2);
        if(result != EXIT_SUCCESS){
            printf("interpreter->get_output_tensor() 2 failed !\n");
            return EXIT_FAILURE;
        }
        printf("sample : interpreter->get_output_tensor() 2 length is [%d] !\n", output_tensor_length_2);

        std::chrono::steady_clock::time_point et2 = std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration dur2 = et2 - st2;
        printf("current thread_idx[%d] [%d] get_output_tensor cost time : %f\n", thread_idx, idx, std::chrono::duration<double>(dur2).count()*1000);
        sum_time_2 += std::chrono::duration<double>(dur2).count()*1000;
    }
    printf("repeat [%d] time , input[%f] --- invoke[%f] --- output[%f] --- sum[%f]ms\n", _counter, sum_time_0, sum_time_1, sum_time_2, sum_time_0+sum_time_1+sum_time_2);

    std::chrono::steady_clock::time_point pps = std::chrono::steady_clock::now();

    filterBoxes.clear();
    objProbs.clear();
    classId.clear();
    int validCount0 = process(stride8, filterBoxes, objProbs, classId, (float*)anchor0, STRIDE8_SIZE, STRIDE8_SIZE, 8, MODEL_SIZE);
    int validCount1 = process(stride16, filterBoxes, objProbs, classId, (float*)anchor1, STRIDE16_SIZE, STRIDE16_SIZE, 16, MODEL_SIZE);
    int validCount2 = process(stride32, filterBoxes, objProbs, classId, (float*)anchor2, STRIDE32_SIZE, STRIDE32_SIZE, 32, MODEL_SIZE);

    int validCount = validCount0 + validCount1 +validCount2;

    std::vector<int> indexArray;
    for (int i = 0; i < validCount; ++i){
        indexArray.push_back(i);
    }

    quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

    std::set<int> class_set(std::begin(classId), std::end(classId));

    for (auto c : class_set) {
        nms(validCount, filterBoxes, classId, indexArray, c, NMS_THRESH);
    }
    
    std::chrono::steady_clock::time_point ppe = std::chrono::steady_clock::now();
    std::chrono::steady_clock::duration durpp = ppe - pps;
    printf("postprocess cost time : %f ms\n", std::chrono::duration<double>(durpp).count()*1000);

    // 数据来源于 SNPE2 FP32 CPU 运行结果 [x1, y1, x2, y2] 坐标向下取整
    const float expected_box_0[3][4] = {{210, 241, 285, 519}, {473, 229, 560, 522}, {108, 231, 231, 542}};
    const float expected_box_5[1][4] = {{91,  131, 551, 464}};

    unsigned int box_count = 0;
    unsigned int verify_pass_count = 0;
    for (int i = 0; i < validCount; ++i) {
        
        if (indexArray[i] == -1) {
            continue;
        }
        int n = indexArray[i];

        float x1       = filterBoxes[n * 4 + 0] * scale;
        float y1       = filterBoxes[n * 4 + 1] * scale;
        float x2       = x1 + filterBoxes[n * 4 + 2] * scale;
        float y2       = y1 + filterBoxes[n * 4 + 3] * scale;
        int   id       = classId[n];
        float obj_conf = objProbs[i];

        string show_info = "class " + to_string(id) + ": " + to_string(obj_conf);
        cv::putText(frame, show_info.c_str(), cv::Point(x1, y1), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, 2);   // color-BGR
        cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2, 2, 0);

        // 结果正确性验证
        printf("Result id[%d]-x1[%f]-y1[%f]-x2[%f]-y2[%f]\n", id, x1, y1, x2, y2);

        ++box_count;
        if(id == 0){
            for(int idx = 0; idx < 3; ++idx){
                float coverage_ratio = CalculateOverlap(x1, y1, x2, y2, 
                            expected_box_0[idx][0], expected_box_0[idx][1], expected_box_0[idx][2], expected_box_0[idx][3]);
                printf("Verify result : idx[%d] id[%d] coverage_ratio[%f]\n", idx, id, coverage_ratio);
                if(coverage_ratio > 0.9){
                    ++verify_pass_count;
                    break;
                }
            }
        }else if(id == 5){
            for(int idx = 0; idx < 1; ++idx){
                float coverage_ratio = CalculateOverlap(x1, y1, x2, y2, 
                            expected_box_5[idx][0], expected_box_5[idx][1], expected_box_5[idx][2], expected_box_5[idx][3]);
                printf("Verify result : idx[%d] id[%d] coverage_ratio[%f]\n", idx, id, coverage_ratio);
                if(coverage_ratio > 0.9){
                    ++verify_pass_count;
                    break;
                }
            }
        }else{
            printf("ERROR : The Yolov5s model inference result is not the expected classification category.\n");
            return EXIT_FAILURE;
        }
    }
    
    // 保存结果图片
    cv::cvtColor(frame, frame , cv::COLOR_RGB2BGR);

    std::string result_display_file_name = save_name+"_trd"+std::to_string(thread_idx)+".jpg";
    cv::imwrite(result_display_file_name, frame);
    printf("The result display file[%s] for the example is located in the current working path\n", result_display_file_name.c_str());

    // 判定结果正确性
    printf("The inference result box_count[%d]-verify_pass_count[%d].\n", box_count, verify_pass_count);
    if((box_count != 4 && box_count != 5) || verify_pass_count < 3){

        // for(int idx = 0; idx < 3; ++idx){
        //     cv::rectangle(frame, cv::Point(expected_box_0[idx][0], expected_box_0[idx][1]), 
        //                         cv::Point(expected_box_0[idx][2], expected_box_0[idx][3]), cv::Scalar(0, 0, 255), 2, 2, 0);
        // }
        // for(int idx = 0; idx < 1; ++idx){
        //     cv::rectangle(frame, cv::Point(expected_box_5[idx][0], expected_box_5[idx][1]), 
        //                         cv::Point(expected_box_5[idx][2], expected_box_5[idx][3]), cv::Scalar(0, 0, 255), 2, 2, 0);
        // }

        // cv::imwrite(result_display_file_name, frame);

        printf("ERROR : The coverage ratio check failed.\n");
        return EXIT_FAILURE;
    }

    result = qnn_interpreter->destory();
    if(result != EXIT_SUCCESS){
        printf("interpreter->destory() failed !\n");
        return EXIT_FAILURE;
    }

    printf("exit thread_func[%d]\n", thread_idx);

    return EXIT_SUCCESS;
}

int main(int argc, char** args)
{
    if(argc != 2){
        show_help_prompt();
        return EXIT_FAILURE;
    }
    
    acc_type = std::stoi(args[1]);

    std::future<int> thread_01_result = std::async(std::launch::async, thread_func, 1);
    // std::future<int> thread_02_result = std::async(std::launch::async, thread_func, 2);
    // std::future<int> thread_03_result = std::async(std::launch::async, thread_func, 3);

    if(EXIT_SUCCESS != thread_01_result.get()){
        printf("ERROR : thread_01 run failed.\n");
        return EXIT_FAILURE;
    }
    // if(EXIT_SUCCESS != thread_02_result.get()){
    //     printf("ERROR : thread_02 run failed.\n");
    //     return EXIT_FAILURE;
    // }
    // if(EXIT_SUCCESS != thread_03_result.get()){
    //     printf("ERROR : thread_03 run failed.\n");
    //     return EXIT_FAILURE;
    // }

    printf("Exit main function .\n");
    return 0;
}