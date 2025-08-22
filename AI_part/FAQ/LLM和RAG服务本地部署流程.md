# LLM服务本地部署

#### 1、下载软件工具

```shell
# 下载 aidllm 安装包
	wget https://aidllm.aidlux.com/files/aidllm_2.0.0_arm64.aid.gpg

# 安装 aidllm
	sudo aid-pkg -i -d aidllm_2.0.0_arm64.aid.gpg

# 下载 aidgen-sdk 安装包
	wget http://192.168.110.22:18080/job/aplux_aidgen/9/artifact/aidgen-sdk_1.0.0.9_arm64_ub2204.aid.gpg

# 安装 aidgen-sdk🔍 查询可⽤模型
	sudo aid-pkg -i -d aidgen-sdk_1.0.0.9_arm64_ub2204.aid.gpg

# 下载 aid-open-api 安装包
	wget https://aidllm.aidlux.com/files/aid-openai-api_2.0.0_arm64.aid.gpg

# 安装 aid-openao-api
	sudo aid-pkg -i -d aid-openai-api_2.0.0_arm64.aid.gpg
```

### 2、部署下载的模型

```shell
# 查看远程支持的模型
aidllm remote-list api
    Current Soc : 8550
    Name Url CreateTime
    
# 拉去远程模型
aidllm pull api aplux/aplux_qwen2.5-3B

# 查看本地已下载模型列表
	aidllm list api

# 加载模型
	aidllm start api -m aplux_qwen2.5-3B

# 安装 UI 前端服务
	sudo aidllm install ui
	sudo aidllm start ui
```

### 3、部署模型的文件结构

✅ **GGUF** 类型模型（跨平台⽀持，推荐模型格式）

###### 	路径：/opt/aidlux/app/aid-openai-api/res/models/qwen2.5-1.5b-instruct-q4_k_m/qwen2.5-1.5b-instruct-q4_k_m.gguf

```shell
推荐⽂件结构如下：
    qwen2.5-1.5b-instruct-q4_k_m/
        ├── qwen2.5-1.5b-instruct-q4_k_m.gguf
        ├── qwen2.5-1.5b-instruct-q4_k_m.json ← 模型配置文件（需手动编写）    
json配置⽂件模板：
{
    "backend_type": "llamacpp",
        "model": {
        	"path": "<模型文件绝对路径>"
        }
}
```



### 4、**Linux** 系统模型上传步骤：

#### **1.** 上传整个模型⽬录（包含多个权重或配置⽂件）

```shell
	scp -r /local/path/to/qwen2.5-1.5b-instruct-q4_k_m user@linux-device-ip:/opt/aidlux/app/aidopen-api/res/models/
```

#### **2.** 本地拷⻉⽅式

```shell
	cp -r qwen2.5-1.5b-instruct-q4_k_m /opt/aidlux/app/aid-open-api/res/models/
```

#### **3.** 将上传的模型⽂件加⼊**aidllm**配置

##### 	1. 打开aidllm的api_cfg.json配置⽂件

```shell
	vim /opt/aidlux/app/aid-openai-api/api-cfg.json
	
	结构内容如下：
	{
        "prompt_template_list":[
            {"qwen1.5":"<|im_start|>system\n{0}<|im_end|>\n<|im_start|>user\n{1}<|im_end|>\n<|im_start|>assistant\n"},
            {"qwen2":"<|im_start|>system\n{0}<|im_end|>\n<|im_start|>user\n{1}<|im_end|>\n<|im_start|>assistant\n"},
            {"deepseek":"<｜begin▁of▁sentence｜>{0}<｜User｜>{1}<｜Assistant｜>"}
        ],
        "model_cfg_list":[
             {
              "model_id": "Qwen3-0.6B-Q8_gguf",
              "model_create": "1750843418159",
              "model_owner": "aplux",
              "cfg_path": "./models/Qwen3-0.6B-Q8_gguf/Qwen3-0.6B-Q8_gguf.json",
              "prompt_template_type": "qwen2"
            }
        ],
        "http_cfg":[
            {"ip":"0.0.0.0"},
            {"port":8888}
        ],
        "work_folder":"/opt/aidlux/app/aid-openai-api",
        "dsp_skel_folder":"",
        "res_folder":"./res",
        "log_level" : "INFO",
        "log_folder" : "./logs/",
        "default_model_id": ""
    }
```

##### 	2.在**model_cfg_list**列表⼿动修改，添加模型配置信息

```shell
字段说明：
model_id ：模型标识，唯⼀ ID，⽤于运⾏时指定模型，例如 qwen2.5-1.5b-instruct-q4_k_m 。

model_create ：模型注册时间戳，可使⽤命令 printf "%s%03d\n" "$(date +%s)" "$((10#$(date +%N)/ 1000000))" ⽣成，单位为毫秒。

model_owner ：模型所有者名称，可⾃定义，例如 "aplux" 。

cfg_path ：模型主配置⽂件路径，Linux 下使⽤相对路径（如 ./models/... ），Android 下需使⽤绝对路径（如 /sdcard/... ）。
		./models/qwen2.5-1.5b-instruct-q4_k_m/qwen2.5-1.5b-instruct-q4_k_m.json
		./models/qwen2.5-1.5B-instruct-8550-bin/qwen2.5-1.5b-instruct-htp.json
		注意一点为添加bin类型模型配置文件时，需要指定例如：qwen2.5-1.5b-instruct-htp.json，而非htp-backend.json

prompt_template_type ：对话提⽰模板类型，常⻅取值如 "qwen1.5" 、 "qwen2" 、 "deepseek" ，应与模型训练格式匹配。

```

##### 	4.使⽤ **aidllm** 命令⾏⼯具查看添加模型

```shell
# 使用aidllm list api命令查看模型是否已经加入aidllm
    aidllm list api
        qwen2.5-7b-instruct
        aplux_qwen2-7B
        qwen2.5-7b-instruct-q4_k_m
        qwen2.5-7B-8550
        qwen2.5-1.5b-instruct-q4_k_m
        qwen2.5-1.5B-instruct-8550-bin

# 使用aidllm start api -m qwen2.5-1.5b-instruct-q4_k_m 指定运行模型，查看模型是否能够运行
    aidllm start api -m qwen2.5-1.5b-instruct-q4_k_m
```





## 知识库RAG（Retrieval-Augmented Generation）服务本地部署

### 1、安装软件工具包

```shell
# 下载 aidllm 安装包
	wget https://aidllm.aidlux.com/files/aidllm_2.0.0_arm64.aid.gpg

# 安装 aidllm
	sudo aid-pkg -i -d aidllm_2.0.0_arm64.aid.gpg

# 下载 aidgen-sdk 安装包
	wget http://192.168.110.22:18080/job/aplux_aidgen/9/artifact/aidgen-sdk_1.0.0.9_arm64_ub2204.aid.gpg
	
# 安装 aidgen-sdk
	sudo aid-pkg -i -d aidgen-sdk_1.0.0.9_arm64_ub2204.aid.gpg
```



### 2、获取aidlux账号

🔐一、注册账号

访问 aidlux（[Profile | AidLux Single Sign-On](https://auth.aidlux.com/zh/user/profile)） 官⽹，注册并登录⼀个 **Aidlux** 账号。

📚 ⼆、创建 **RAG** 知识库

   1.🌐 登录 **aidllm-cms**

​	打开浏览器访问 👉 https://aidllm.aidlux.com，使⽤你的 Aidlux 账号登录

   2.🧭 进⼊知识库管理界⾯

​	登录后点击左侧“知识库”菜单，进⼊管理⻚⾯，点击“新建”。

​    3.📝 填写知识库信息

​	名称：仅⽀持英⽂字⺟和数字（如： MyRAG2025 ）。

​	嵌⼊模型：选择当前已加载的 embedding 模型。

​	切⽚⽅法：可选 General 或 Q&A ，具体说明如下：

🧩 切⽚⽅法 	📄 ⽀持格式 		📖 说明

General		⽂本（.txt /.pdf）	将连续⽂本按“分段标识符”分割，再按Token 数量不超过“最⼤⻓度”合并为⼀块。

Q&A			.xlsx / .csv /.txt	  ⽤于问答格式：Excel 两列（⽆表头：问题 / 答案）；CSV / TXT 使⽤ **Tab** 分隔，UTF-8 编码。



💻 三、命令⾏操作 **aidllm**

1. 🔑 登录远程 **cms**

```shell
	aidllm login
```

2. 安装RAG服务

   ```
   sudo aidllm install rag
   ```

3. 📃 查看知识库列表

   ```shell
   aidllm remote-list rag
   
       Name EmbeddingModel CreateTime
       aidluxdocs BAAI/bge-large-zh-v1.5 2025-07-09 14:56:31
   ```

4. 拉取知识库和启动

   ```shell
   aidllm pull rag <知识库名称>
   aidllm start rag
   aidllm start rag -n <知识库名称>
   ```
   
   







