#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <string>
#include <queue>
#include <filesystem>
#include <vector>
#include <stack>
#include <optional>
#include <regex>
#include <map>
#include <cmath>

#include "run_options.h"
#include "angle_options.h"
#include "kernel_options.h"
#include "noise_options.h"


namespace po = boost::program_options;

using namespace cv;
using namespace std;
using std::filesystem::directory_iterator;

//fixed parameters
//const int blurKsizes[] = {3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29};
const double noiseDeviations[] = {2, 4, 8, 16, 32, 64, 128};
//const double ANGLES[] = {7.0, 40.0, 78.0, 122.0};
const string SETS[] = {"DRIVE", "CHASEDB1", "STARE", "HRF"};

class BlurKernelRepository{
private:
    map<string, map<int, Mat>> m;
public:
    Mat& get(string s, int k){
        return m[s][k];
    }
    BlurKernelRepository(AngleOptions &aopts, KernelOptions &kopts, NoiseOptions &nopts){
//        string mod = "mbH";
        string mod = RunOption::MOT_BLUR_H;
        m[mod] = map<int, Mat>();
        for(int k : kopts.get_kernels()) {
            Mat work(k, k, CV_32FC1);
            work.setTo(0.0);
            //line(work, Point(0, k/2), Point(k, k/2), 255, 1);
            line(work, Point(0, k/2), Point(k, k/2), 1.0, 1, LineTypes::LINE_8);
            GaussianBlur(work, work, Size(3, 3), 0);
            //work = work * 255.0;
            //Mat rez(k, k, CV_8UC1);
            //work.convertTo(rez, CV_8UC1);
            //work = work * (1.0 / (double)k);
            m[mod][k] = work / sum(work);
        }
//        mod = "mbV";
        mod = RunOption::MOT_BLUR_V;
        m[mod] = map<int, Mat>();
        for(int k : kopts.get_kernels()){
            Mat work(k*2, k*2, CV_32FC1);
            work.setTo(0);
            line(work, Point(k, 0), Point(k, 2*k), 1.0, 2);
            GaussianBlur(work, work, Size(3, 3), 0);
            Mat rez(k, k, CV_32FC1);
            resize(work, rez, rez.size());
            rez = rez / k;
            m[mod][k] = rez / sum(rez);
        }
//        mod = "mbC";
        mod = RunOption::MOT_BLUR_C;
        m[mod] = map<int, Mat>();
        for(int k : kopts.get_kernels()){
            int i = 0;
            for(double a : aopts.get_angles()){
                double arad = (a / 360.0) * 2 * M_PI;
                if(arad > M_PI) arad -= M_PI;
                Mat work(k*2, k*2, CV_32FC1);
                work.setTo(0.0);
                if(arad > M_PI/4 && arad < 3*M_PI/4){
                    line(work, Point(k - k*tan(M_PI/2 - arad), 0), Point(k + k*tan(M_PI/2 - arad), 2*k), 1.0, 2);
                }else{
                    line(work, Point(0, k + k*tan(arad)), Point(2*k, k - k*tan(arad)), 1.0, 2);
                }
                GaussianBlur(work, work, Size(3, 3), 0);
                Mat rez(k, k, CV_32FC1);
                resize(work, rez, rez.size());
                rez = rez / k;
                m[mod][(i << 16) | k] = rez / sum(rez);
                i++;
            }
        }
    }
};

enum RequestType {
    IMAGE,
    GROUND,
    MASK,
    TERMINAL,
    MOD,
    SKIP
};

struct Request{
    string path;
    RequestType type;
};

bool isDataSet(string p){
    for(const auto& s : SETS){
        if(s == p) return true;
    }
    return false;
}

RequestType getTypeFromPath(std::filesystem::path p){
    const int S_DATA = 0;
    const int S_SUBSET = 1;
    const int S_TYPE = 2;
    int state = S_DATA;
    for(auto a : p){
        switch(state){
            case S_DATA:
                if(isDataSet(a.string())){
                     state = S_SUBSET;
                }
                if(a.string() == "HDF5") return SKIP;
                break;
            case S_SUBSET:
                state = S_TYPE;
                break;
            case S_TYPE:
                if(a.string() == "1st_manual") return GROUND;
                else if(a.string() == "images") return IMAGE;
                else if(a.string() == "mask") return MASK;
                else if(a.string() == "labels-ah") return GROUND;
                else if(a.string() == "stare-images") return IMAGE;
                else return SKIP;
                break;
        }
    }
    return SKIP;
}

string getN(string dataset, string src){
    static regex chasedb("Image_(\\d\\d.).*");
    static regex drive("^(\\d\\d).*");
    static regex hrf("^(\\d\\d_[a-z]{1,2}).*");
    static regex stare("^im(\\d{4}).*");
    smatch m;
    if(dataset == "CHASEDB1"){
        regex_match(src, m, chasedb);
        if(m.size() == 2){
            return m[1].str();
        }else{
            //something's gone terribly wrong
            cerr << "Unexpected filename in CHASEDB1 parsing. " << src << endl;
            exit(1); 
        }
    }else if(dataset == "DRIVE"){
        regex_match(src, m, drive);
        if(m.size() == 2){
            return m[1].str();
        }else{
            //something's gone terribly wrong
            cerr << "Unexpected filename in DRIVE parsing." << endl;
            exit(1); 
        }
    }else if(dataset == "HRF"){
        regex_match(src, m, hrf);
        if(m.size() == 2){
            return m[1].str();
        }else{
            //something's gone terribly wrong
            cerr << "Unexpected filename in HRF parsing." << endl;
            exit(1); 
        }
    }else if(dataset == "STARE"){
        regex_match(src, m, stare);
        if(m.size() == 2){
            return m[1].str();
        }else{
            //something's gone terribly wrong
            cerr << "Unexpected filename in STARE parsing." << endl;
            exit(1); 
        }
    }else{
        //unknown dataset, firewall code
        return src;
    }
}

string getExt(string type, string src){
    static regex ext("^.*\\.(\\w\\w\\w)$");
    smatch m;
    if(type == "mod"){
        return "png";
    }else{
        regex_match(src, m, ext);
        if(m.size() == 2){
            return m[1].str();
        }else{
            //something's gone terribly wrong
            cerr << "Unexpected filename in extension parsing." << endl;
            exit(1);
        }
    }
}

optional<string> getOutputName(std::filesystem::path p, optional<string> mod, optional<string> params, bool useDatasets){
    const int S_DATA = 0;
    const int S_SUBSET = 1;
    const int S_TYPE = 2;
    const int S_N = 3;
    const int S_DONE = 4;
    int state = S_DATA;
    if(!useDatasets){
        if(mod.has_value() && params.has_value()){
            static regex splitFile("^(.*)\\.(\\w\\w\\w)$");
            smatch m;
            auto s = p.filename().string();
            regex_match(s, m, splitFile);
            if(m.size() == 3){
                string ret = (boost::format("%1%-%2%-%3%.%4%") % m[1].str() % mod.value() % params.value() % m[2].str()).str();
                return ret;
            }else{
                cerr << "Unexpected filename in extension parsing." << endl;
                exit(1);
            }
        }else{
            return p.filename().string();
        }
    }
    string data;
    string subset;
    string n;
    string type; 
    string ext;
    for(auto a : p){
        switch(state){
            case S_DATA:
                if(isDataSet(a.string())){
                     state = S_SUBSET;
                     data = a.string();
                }
                if(a.string() == "HDF5") return {};
                break;
            case S_SUBSET:
                state = S_TYPE;
                subset = a.string();
                break;
            case S_TYPE:
                state = S_N;
                if (mod.has_value()) type = "mod";
                else if(a.string() == "1st_manual") type = "gnd";
                else if(a.string() == "images") type = "img";
                else if(a.string() == "mask") type = "msk";
                else if(a.string() == "labels-ah") type = "gnd";
                else if(a.string() == "stare-images") type = "img";
                else return {};
                break;
            case S_N:
                n = getN(data, a.string());
                ext = getExt(type, a.string());
                state = S_DONE;
                break;                
        }
    }
    if(state == S_DONE){
        if(mod.has_value() && params.has_value()){
            return (boost::format("%1%-%2%-%3%-%4%-%5%-%6%.%7%") % data % subset % n % "mod" % mod.value() % params.value() % ext).str();
        }else{
            return (boost::format("%1%-%2%-%3%-%4%-0-0.%5%") % data % subset % n % type % ext).str();
        }
    }else{
        return {};
    }
}

mutex terminal;

class RequestList{
    private:
        queue<Request> requests;
        mutex mx;
        condition_variable notEmpty;
        bool terminate;
        bool terminated;
    public:
        RequestList() : terminate(false), terminated(false){}
        void store(Request r){
            unique_lock<mutex> l(mx);
            requests.push(r);
            notEmpty.notify_one();
        }
        Request load(){
            unique_lock<mutex> l(mx);
            while(requests.size() == 0 && terminated == false){
                notEmpty.wait(l);
            }
            if(terminated) return Request{.path = "", .type = RequestType::TERMINAL};
            Request ret = requests.front();
            requests.pop();
            if(terminate && requests.empty()){
                terminated = true;
                notEmpty.notify_all();
            }
            return ret;
        }
        void doTerminate(){
            unique_lock<mutex> l(mx);
            terminate = true;
            if(requests.empty()){
                terminated = true;
                notEmpty.notify_all();
            }
        }
};

string formatParamMap(map<string, string>& p){
    string ret;
    int n = p.size();
    int i = 0;
    for(auto e : p){
        ret.append(e.first);
        ret.append("_");
        ret.append(e.second);
        i++;
        if(i != n){
            ret.append("_");
        }
    }
    return ret;
}

void processRequest(RequestList& rl, string& outputPath, bool useDatasets, bool copyUnchanged, RunOption &ropts,
        AngleOptions &aopts, KernelOptions &kopts, NoiseOptions &nopts, BlurKernelRepository& bkr){
    while(1){
        Request r = rl.load();
        if(r.type == RequestType::TERMINAL) return;
        if(r.type == RequestType::SKIP) continue;
        filesystem::path p(r.path);
        filesystem::path op(outputPath);
        filesystem::path o;
        if(r.type == IMAGE){
            auto on = getOutputName(p, {}, {}, useDatasets);
            if(!on.has_value()) continue;
            o = op / filesystem::path(on.value());
            {
                unique_lock<mutex> l(terminal);
                cout << "Processing " << r.path << " into " << o << " as image." << endl;
            }
            if(copyUnchanged){
                filesystem::copy_file(p, o);
            }else{
                filesystem::create_symlink(p, o);
            }
            map<string, string> params;
            Mat img = imread(p,ImreadModes::IMREAD_COLOR);
            if(!img.data){
                unique_lock<mutex> l(terminal);
                cerr << "Error reading image @ " << p << endl;
                continue;
            }
            Mat imgYUV(img.rows, img.cols, img.type());
            Mat imgHD(img.rows, img.cols, CV_64FC3);
            img.convertTo(imgHD, CV_64FC3, 1/255.0);
            {
                int w = imgHD.size().width;
                int h = imgHD.size().height;
                resize(imgHD, imgHD, Size(w + w % 2 , h + h % 2));
            }
            cvtColor(img, imgYUV, cv::COLOR_BGR2YUV);
            vector<Mat> lRGB;
            vector<Mat> lYUV;
            vector<Mat> lHD;
            split(img, lRGB);
            split(imgYUV, lYUV);
            split(imgHD, lHD);
            vector<Mat> ldst;
            vector<Mat> ldstHD;
            ldst.push_back(Mat(img.rows, img.cols, CV_8UC1));
            ldst.push_back(Mat(img.rows, img.cols, CV_8UC1));
            ldst.push_back(Mat(img.rows, img.cols, CV_8UC1));

            ldstHD.push_back(Mat(imgHD.rows, imgHD.cols, CV_64FC1));
            ldstHD.push_back(Mat(imgHD.rows, imgHD.cols, CV_64FC1));
            ldstHD.push_back(Mat(imgHD.rows, imgHD.cols, CV_64FC1));
            

            Mat dst(img.rows, img.cols, img.type());
            Mat dstHD(imgHD.rows, imgHD.cols, CV_64FC3);
            Mat noise(img.rows, img.cols, CV_8SC1);
            Mat noiseHD(imgHD.rows, imgHD.cols, CV_64FC1);

            vector<int> blurKsizes = kopts.get_kernels();

            // apply gaussian blur on an image
            if (ropts.find_option(RunOption::GAUSS_UNI_BLUR)) {
                cout << "[processRequest] Generating " << RunOption::GAUSS_UNI_BLUR << "..." << endl;
                for(int k : blurKsizes){
                    GaussianBlur(img, dst, Size(k,k), 0.0);
                    params.clear();
                    params["ksize"] = to_string(k);
                    auto on = getOutputName(p, "gb", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dst);
                }
            }

            // apply custom motion blur
            if (ropts.find_option(RunOption::MOT_BLUR_C)) {
                string mod = RunOption::MOT_BLUR_C;
                cout << "[processRequest] Generating " << RunOption::MOT_BLUR_C << "..." << endl;
                for(int k : blurKsizes) {
                    int i = 0;
                    for (double a : aopts.get_angles()) {
                        filter2D(img, dst, -1, bkr.get(mod, (i << 16) | k));
                        params.clear();
                        params["ksize"] = to_string(k);
                        params["angle"] = to_string(lround(a));
                        auto on = getOutputName(p, "mbC", formatParamMap(params), useDatasets);
                        o = op / filesystem::path(on.value());
                        imwrite(o, dst);
                        i++;
                    }
                }
            }

            // apply horizontal and vertical motion blur
            if (ropts.find_option(RunOption::MOT_BLUR_H)) {
                string mod = RunOption::MOT_BLUR_H;
                cout << "[processRequest] Generating " << mod << "..." << endl;
                for(int k : blurKsizes) {
                    filter2D(img, dst, -1, bkr.get(mod, k));
                    params.clear();
                    params["ksize"] = to_string(k);
                    auto on = getOutputName(p, "mbH", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dst);
                }
            }


            // apply horizontal and vertical motion blur
            if (ropts.find_option(RunOption::MOT_BLUR_V)) {
                string mod = RunOption::MOT_BLUR_V;
                cout << "[processRequest] Generating " << mod << "..." << endl;
                for(int k : blurKsizes) {
                    filter2D(img, dst, -1, bkr.get(mod, k));
                    params.clear();
                    params["ksize"] = to_string(k);
                    auto on = getOutputName(p, "mbV", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dst);
                }
            }

            vector<double> noiseDeviations = nopts.get_noise_deviations();
            // apply noise on the image, mono
            if (ropts.find_option(RunOption::GAUSS_NOISE_MONO)) {
                cout << "[processRequest] Generating " << RunOption::GAUSS_NOISE_MONO << "..." << endl;
                for(double sd : noiseDeviations){
                    lRGB[0].copyTo(ldst[0]);
                    lRGB[1].copyTo(ldst[1]);
                    lRGB[2].copyTo(ldst[2]);
                    randn(noise, 0, sd);
                    ldst[0] += noise;
                    ldst[1] += noise;
                    ldst[2] += noise;
                    merge(ldst, dst);
                    params.clear();
                    params["sd"] = to_string(lround(sd));
                    auto on = getOutputName(p, "gnoise_all_mono", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dst);
                }
            }

            // apply noise on all image channels
            if (ropts.find_option(RunOption::GAUSS_NOISE)) {
                cout << "[processRequest] Generating " << RunOption::GAUSS_NOISE << "..." << endl;
                for(double sd : noiseDeviations){
                    lRGB[0].copyTo(ldst[0]);
                    lRGB[1].copyTo(ldst[1]);
                    lRGB[2].copyTo(ldst[2]);
                    randn(noise, 0, sd);
                    ldst[0] += noise;
                    randn(noise, 0, sd);
                    ldst[1] += noise;
                    randn(noise, 0, sd);
                    ldst[2] += noise;
                    merge(ldst, dst);
                    params.clear();
                    params["sd"] = to_string(lround(sd));
                    auto on = getOutputName(p, "gnoise_all_color", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dst);
                }
            }

            // apply noise on the image's blue channel
            if (ropts.find_option(RunOption::GAUSS_NOISE_B)) {
                cout << "[processRequest] Generating " << RunOption::GAUSS_NOISE_B << "..." << endl;
                for(double sd : noiseDeviations){
                    lRGB[0].copyTo(ldst[0]);
                    lRGB[1].copyTo(ldst[1]);
                    lRGB[2].copyTo(ldst[2]);
                    randn(noise, 0, sd);
                    ldst[0] += noise;
                    merge(ldst, dst);
                    params.clear();
                    params["sd"] = to_string(lround(sd));
                    auto on = getOutputName(p, "gnoise_b", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dst);
                }
            }

            // apply noise on the image's green channel
            if (ropts.find_option(RunOption::GAUSS_NOISE_G)) {
                cout << "[processRequest] Generating " << RunOption::GAUSS_NOISE_G << "..." << endl;
                for(double sd : noiseDeviations){
                    lRGB[0].copyTo(ldst[0]);
                    lRGB[1].copyTo(ldst[1]);
                    lRGB[2].copyTo(ldst[2]);
                    randn(noise, 0, sd);
                    ldst[1] += noise;
                    merge(ldst, dst);
                    params.clear();
                    params["sd"] = to_string(lround(sd));
                    auto on = getOutputName(p, "gnoise_g", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dst);
                }
            }

            // apply noise on the image's red channel
            if (ropts.find_option(RunOption::GAUSS_NOISE_R)) {
                cout << "[processRequest] Generating " << RunOption::GAUSS_NOISE_R << "..." << endl;
                for(double sd : noiseDeviations){
                    lRGB[0].copyTo(ldst[0]);
                    lRGB[1].copyTo(ldst[1]);
                    lRGB[2].copyTo(ldst[2]);
                    randn(noise, 0, sd);
                    ldst[2] += noise;
                    merge(ldst, dst);
                    params.clear();
                    params["sd"] = to_string(lround(sd));
                    auto on = getOutputName(p, "gnoise_r", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dst);
                }
            }

            // apply noise on the image in YUV colorspace
            if (ropts.find_option(RunOption::GAUSS_NOISE_YUV)) {
                cout << "[processRequest] Generating " << RunOption::GAUSS_NOISE_YUV << "..." << endl;
                for(double sd : noiseDeviations){
                    lYUV[0].copyTo(ldst[0]);
                    lYUV[1].copyTo(ldst[1]);
                    lYUV[2].copyTo(ldst[2]);
                    randn(noise, 0, sd);
                    ldst[1] += noise;
                    randn(noise, 0, sd);
                    ldst[2] += noise;
                    merge(ldst, dst);
                    params.clear();
                    params["sd"] = to_string(lround(sd));
                    cvtColor(dst, dst, cv::COLOR_YUV2BGR);
                    auto on = getOutputName(p, "gnoise_yuv_color", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dst);
                }
            }

            // apply noise on the image in frequent domain
            if (ropts.find_option(RunOption::GAUSS_NOISE_FREQ)) {
                cout << "[processRequest] Generating " << RunOption::GAUSS_NOISE_FREQ << "..." << endl;
                for(double sd : noiseDeviations){
                    dct(lHD[0], ldstHD[0]);
                    dct(lHD[1], ldstHD[1]);
                    dct(lHD[2], ldstHD[2]);

                    randn(noiseHD, 0, sd / 128.0);
                    ldstHD[0] += noiseHD;
                    randn(noiseHD, 0, sd / 128.0);
                    ldstHD[1] += noiseHD;
                    randn(noiseHD, 0, sd / 128.0);
                    ldstHD[2] += noiseHD;

                    dct(ldstHD[0], lHD[0], DCT_INVERSE);
                    dct(ldstHD[1], lHD[1], DCT_INVERSE);
                    dct(ldstHD[2], lHD[2], DCT_INVERSE);

                    merge(lHD, dstHD);
                    params.clear();
                    params["sd"] = to_string(lround(sd));
                    auto on = getOutputName(p, "gnoise_freq", formatParamMap(params), useDatasets);
                    o = op / filesystem::path(on.value());
                    imwrite(o, dstHD * 255.0);
                }
            }

        }else if(r.type == GROUND){
            auto on = getOutputName(p, {}, {}, useDatasets);
            if(!on.has_value()) continue;
            o = op / filesystem::path(on.value());
            {
                unique_lock<mutex> l(terminal);
                cout << "Processing " << r.path << " into " << o << " as ground truth." << endl;
            }
            if(copyUnchanged){
                filesystem::copy_file(p, o);
            }else{
                filesystem::create_symlink(p, o);
            }
        }else if(r.type == MASK){
            auto on = getOutputName(p, {}, {}, useDatasets);
            if(!on.has_value()) continue;
            o = op / filesystem::path(on.value());
            {
                unique_lock<mutex> l(terminal);
                cout << "Processing " << r.path << " into " << o << " as mask." << endl;
            }
            if(copyUnchanged){
                filesystem::copy_file(p, o);
            }else{
                filesystem::create_symlink(p, o);
            }
        }else{
            //NOOP
        }
    }
}

void feedList(RequestList& rl, string& inputPath, bool useDatasets){
    if(useDatasets){
        stack<filesystem::path> toProcess;
        toProcess.push(filesystem::path(inputPath));
        while(!toProcess.empty()){
            filesystem::path p = toProcess.top();
            toProcess.pop();
            for(auto & file : directory_iterator(p)){   
                if(file.is_directory()){
                    toProcess.push(file);
                }else{
                    auto t = getTypeFromPath(file.path());
                    if(t != SKIP){
                        rl.store(Request {.path = file.path().string(), .type = t});
                    }
                }
            }
        }
        rl.doTerminate();
    }else{
        for(const auto & file : directory_iterator(inputPath)){
            if(!file.is_directory()){
                Request r {.path=file.path(), .type=RequestType::IMAGE};
                rl.store(r);
            }
        }
        rl.doTerminate();        
    }
}


void conflicting_options(const po::variables_map& vm, const char* opt1, const char* opt2)
{
    if (vm.count(opt1) && !vm[opt1].defaulted() && vm.count(opt2) && !vm[opt2].defaulted())
        throw logic_error(string("Conflicting options '") + opt1 + "' and '" + opt2 + "'.");
}


int main(int argc, char** argv){
    //Option values
    int threads;
    string inputPath;
    string outputPath;
    bool useDatasets = false;
    bool copyUnchanged = false;
    RunOption ropts;
    double start_angle, end_angle, step_angle;
    vector<double> angles = vector<double>();
    int start_kernel, end_kernel, step_kernel;
    vector<int> kernels = vector<int>();
    double start_noise_dev, end_noise_dev, step_noise_dev;
    vector<double> noise_devs = vector<double>();

    //Options Parsing Part
    po::options_description desc("Program options:");
    desc.add_options()
        ("help", "Prints this message.")
        ("threads", po::value<int>(&threads)->default_value(1),
                "Number of threads to run. Defaults to 1.")
        ("inputPath,I", po::value<string>(&inputPath)->default_value("."),
                "A source directory. Where to find the images to augment.")
        ("outputPath,O", po::value<string>(&outputPath)->default_value("."),
                "A target directory. Where to store augmented images.")
        ("useDatasets",
                "Predefined directory setup for retinal image datasets. DRIVE, STARE, and CHASE are supported.")
        ("copyUnchanged",
                "Should images for augmentation be copied or linked? Default: linked.")
        ("runOptions,R", po::value<RunOption>(&ropts),
                "What transformations to apply on images. By default, all images from inputPath are augmented using "
                "GAUSSIAN NOISE.")
        ("angles,A", po::value<vector<double>>(&angles)->multitoken(),
                "A list of angles to be used for custom blur transformation. When using this option, options startAngle, "
                "stepAngle, and stepAngle should be omitted.")
        ("startAngle", po::value<double>(&start_angle)->default_value(0),
                "A start angle. Corresponds to the beginning of the kernel range.")
        ("endAngle", po::value<double>(&end_angle)->default_value(120),
                "An end angle. Corresponds to the end of the kernel range.")
        ("stepAngle", po::value<double>(&step_angle)->default_value(40),
                "Step value that is used to iteratively increase startAngle value until endAngle is reached.")
        ("kernels", po::value<vector<int>>(&kernels)->multitoken(),
                "A list containing blur kernel sizes. E.g. [3, 5, 7] means that images will be blurred using kernels of "
                "size 3, 5, and 7 respectively. When using this option, options startKernel, stepKernel, and endKernel "
                "should be omitted.")
        ("startKernel", po::value<int>(&start_kernel)->default_value(3),
                "A start kernel size. Corresponds to the beginning of the kernel range.")
        ("endKernel", po::value<int>(&end_kernel)->default_value(2),
                "An end kernel size. Corresponds to the end of the kernel range.")
        ("stepKernel", po::value<int>(&step_kernel)->default_value(29),
                "Step value that is used to iteratively increase startKernel value until endKernel value is reached.")
        ("noiseDeviations", po::value<vector<double>>(&noise_devs)->multitoken(),
                "A list of standard deviation values that are used for noise transformations. Values should be separated"
                " by comma. When using this option, options startNoiseDev, endNoiseDev and stepNoiseDev should be omitted.")
        ("startNoiseDev", po::value<double>(&start_noise_dev)->default_value(2),
                "A start value of standard deviation. Corresponds to the beginning of the standard deviation range.")
        ("endNoiseDev", po::value<double>(&end_noise_dev)->default_value(2),
                "An end standard deviation value. Corresponds to the end of the standard deviation range.")
        ("stepNoiseDev", po::value<double>(&step_noise_dev)->default_value(10),
                "Step value that is used to iteratively increase startNoiseDev value until endNoiseDev value is reached.")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    conflicting_options(vm, "angles", "startAngle");
    conflicting_options(vm, "angles", "endAngle");
    conflicting_options(vm, "angles", "stepAngle");
    conflicting_options(vm, "kernels", "startKernel");
    conflicting_options(vm, "kernels", "endKernel");
    conflicting_options(vm, "kernels", "stepKernel");
    conflicting_options(vm, "noise_devs", "startNoiseDev");
    conflicting_options(vm, "noise_devs", "endNoiseDev");
    conflicting_options(vm, "noise_devs", "stepNoiseDev");


    po::notify(vm);

    if(vm.count("useDatasets")) useDatasets = true;
    if(vm.count("copyUnchanged")) copyUnchanged = true;


    if (vm.count("help")){
        cout << desc << "\n";
        return 1;
    }

    //threading setup
    AngleOptions aopts(start_angle, end_angle, step_angle, angles);
    KernelOptions kopts(start_kernel, end_kernel, step_kernel, kernels);
    NoiseOptions nopts(start_noise_dev, end_noise_dev, step_noise_dev, noise_devs);
    mutex m;
    condition_variable threadFree;
    int freeThreds = threads;
    RequestList rl; 
    BlurKernelRepository bkr(aopts, kopts, nopts);
    thread fsRunner(feedList, ref(rl), ref(inputPath), useDatasets);
    vector<thread> procs;
    for(int i = 0; i < threads; i++){
        procs.push_back(thread(processRequest, ref(rl), ref(outputPath), useDatasets, copyUnchanged, ref(ropts),
                ref(aopts), ref(kopts), ref(nopts), ref(bkr)));
    }
    for(auto & t : procs){
        t.join();
    }
    fsRunner.join();

    return 0;
}