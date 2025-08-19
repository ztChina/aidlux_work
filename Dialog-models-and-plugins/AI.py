import re
import sys
sys.path.append("/root/.wukong/custom")
from data_publisher import get_publisher


class AICarRobot(AbstractRobot):
    """
    AI小车控制机器人
    基于OpenAI模型进行意图理解和代码生成
    使用HTTP请求方式调用API
    """
    
    SLUG = "ai_car"
    
    def __init__(self, 
                 openai_api_key,
                 model="gpt-3.5-turbo",
                 provider="openai",
                 api_version="2023-12-01-preview",
                 temperature=0.1,
                 max_tokens=1000,
                 top_p=1.0,
                 frequency_penalty=0.0,
                 presence_penalty=0.0,
                 stop_ai=None,
                 prefix="",
                 proxy="",
                 api_base="",
                 **kwargs):
        """
        初始化AI小车机器人
        """
        super(self.__class__, self).__init__()
        
        # 配置参数
        self.openai_api_key = openai_api_key or os.getenv("OPENAI_API_KEY")
        self.model = model
        self.provider = provider
        self.api_version = api_version
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.top_p = top_p
        self.frequency_penalty = frequency_penalty
        self.presence_penalty = presence_penalty
        self.stop_ai = stop_ai
        self.prefix = prefix
        self.proxy = proxy
        self.api_base = api_base if api_base else "https://api.openai.com/v1/chat"
        
        # 对话上下文
        self.context = []
        self.chat_history = []  # 添加chat_history属性以保持兼容性
        
        # 设置系统消息和知识库
        self._setup_conversation()
        
        logger.info(f"AI小车机器人初始化完成，使用模型: {self.model}，提供商: {self.provider}")
    
    @classmethod
    def get_config(cls):
        """获取配置"""
        return config.get("ai_car", {})
    
    @classmethod
    def get_instance(cls):
        """获取实例"""
        profile = cls.get_config()
        instance = cls(**profile)
        return instance
    
    def _setup_conversation(self):
        """设置对话系统"""
        # 添加系统消息
        system_message = """
        你是一个帮助我控制机器小车的智能助手。
        当我要求你做某事时，你需要提供实现该任务所需的Python代码（仅使用CarController类和相关函数），并解释代码的功能。
        你只能使用我已经定义的函数，不能假设或使用任何你认为可能存在的其他函数。

        回复均按照如下格式：
        # 这里输出python代码
        move_forward(2.0)

        此代码功能为：控制小车前进2米。
        你无需考虑car变量的import问题，在环境中已经声明为CarController实例。
        """
        
        self.context.append({
            "role": "system",
            "content": system_message
        })
        
        # 添加知识库
        self._add_knowledge_base()
    
    def _add_knowledge_base(self):
        """添加小车控制函数知识库"""
        knowledge_prompt = """
        以下是您可以用来控制机器小车的所有可用函数：

        移动控制：
        - move_forward(distance) - 前进指定距离(米)
        - move_backward(distance) - 后退指定距离(米)
        - turn_left(angle) - 左转指定角度(度)
        - turn_right(angle) - 右转指定角度(度)
        - follow_point(points) - 移动到指定位置坐标 [x, y]

        请根据用户的指令，选择合适的函数组合来完成任务。
        """
        
        self.context.append({
            "role": "user",
            "content": knowledge_prompt
        })
        
        # 模拟AI确认知识库
        self.context.append({
            "role": "assistant", 
            "content": "我已经了解了所有可用的小车控制函数。我会根据您的指令生成相应的Python代码来控制小车。请告诉我您希望小车执行什么操作。"
        })
    
    def chat(self, texts, parsed=None):
        """
        与AI进行对话 - 兼容wukong-robot接口
        
        :param texts: 用户输入的文本列表
        :param parsed: 解析结果（可选）
        :return: AI的回复
        """
        # 合并输入文本
        user_input = "".join(texts).strip()
        user_input = utils.stripPunctuation(user_input)
        user_input = self.prefix + user_input  # 增加前缀
        
        if not user_input:
            return "请告诉我您希望小车执行什么操作"
        
        logger.info(f"AI小车接收指令: {user_input}")
        
        try:
            # 执行小车控制指令
            result = self.execute_car_command(user_input)
            
            # 返回结果
            if result["execution_result"]:
                return f"{result['ai_response']}\n\n执行结果: {result['execution_result']}"
            elif result["error"]:
                return f"{result['ai_response']}\n\n错误: {result['error']}"
            else:
                return result["ai_response"]
                
        except Exception as e:
            error_msg = f"AI小车控制失败: {e}"
            logger.error(error_msg)
            return error_msg
    
    def stream_chat(self, texts):
        """
        流式对话 - 兼容wukong-robot接口
        
        :param texts: 用户输入的文本列表
        :return: 生成器，逐步返回回复内容
        """
        response = self.chat(texts)
        
        # 模拟流式输出
        words = response.split()
        for word in words:
            yield word + " "
    
    def _chat_with_openai(self, prompt):
        """
        与OpenAI进行对话 - 使用HTTP请求
        
        :param prompt: 用户输入的提示
        :return: AI的回复
        """
        # 添加用户消息到上下文
        self.context.append({
            "role": "user",
            "content": prompt
        })
        
        try:
            # 设置请求头
            header = {
                "Content-Type": "application/json"
            }
            
            if self.provider == 'openai':
                header['Authorization'] = "Bearer " + self.openai_api_key
            elif self.provider == 'azure':
                header['api-key'] = self.openai_api_key
            else:
                raise ValueError("AI小车配置错误，provider应为openai或azure")
            
            # 设置请求数据
            data = {
                "model": self.model,
                "messages": self.context[-10:],  # 保留最近10轮对话
                "temperature": self.temperature,
                "max_tokens": self.max_tokens,
                "top_p": self.top_p,
                "frequency_penalty": self.frequency_penalty,
                "presence_penalty": self.presence_penalty
            }
            
            if self.stop_ai:
                data["stop"] = self.stop_ai
            
            # 设置请求URL
            url = self.api_base + "/completions"
            if self.provider == 'azure':
                url = f"{self.api_base}/openai/deployments/{self.model}/chat/completions?api-version={self.api_version}"
            
            logger.info(f"AI小车使用模型：{self.model}，开始请求")
            
            # 发送HTTP请求
            proxies = {"https": self.proxy} if self.proxy else None
            response = requests.post(
                url,
                headers=header,
                json=data,
                proxies=proxies
            )
            
            if response.status_code == 200:
                response_data = response.json()
                if "choices" in response_data and len(response_data["choices"]) > 0:
                    content = response_data["choices"][0]["message"]["content"]
                    
                    # 添加AI回复到上下文
                    self.context.append({
                        "role": "assistant",
                        "content": content
                    })
                    
                    return content
                else:
                    return "AI小车回复格式错误"
            else:
                error_msg = f"AI小车HTTP请求失败: {response.status_code} - {response.text}"
                logger.error(error_msg)
                return error_msg
            
        except requests.exceptions.RequestException as e:
            error_msg = f"AI小车网络请求失败: {e}"
            logger.error(error_msg)
            return error_msg
        except Exception as e:
            error_msg = f"AI小车API调用失败: {e}"
            logger.error(error_msg)
            return error_msg
    
    def extract_python_code(self, content):
        """
        解析python代码
        
        :param content: AI回复内容
        :return: 提取的Python代码
        """
        code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
        code_blocks = code_block_regex.findall(content)
        
        if code_blocks:
            full_code = "\n".join(code_blocks)
            
            if full_code.startswith("python"):
                full_code = full_code[7:]
            
            return full_code.strip()
        else:
            return None
    
    def execute_car_command(self, user_input):
        """
        处理用户的小车控制指令并发布通知
        
        :param user_input: 用户输入的指令
        :return: 包含处理结果的字典
        """
        logger.info(f"AI小车接收指令: {user_input}")
        
        # 获取AI回复
        ai_response = self._chat_with_openai(user_input)
        logger.info(f"AI小车回复: {ai_response}")
        
        # 提取代码
        code = self.extract_python_code(ai_response)
        
        result = {
            "user_input": user_input,
            "ai_response": ai_response,
            "extracted_code": code,
            "notification_result": None,
            "execution_result": None,
            "error": None
        }
        
        if code:
            logger.info(f"提取的代码: {code}")
            
            try:
                # 发布通知，将提取的代码作为消息
                publisher = get_publisher()
                while True:
                    message =  "status 通知:" + code  # 将提取出的代码作为消息
                    if publisher.send_message(message):
                        break
                
                result["notification_result"] = "代码通知发送成功"
                logger.info("AI小车代码通知发送成功")
                
            except Exception as e:
                error_msg = f"代码通知发送失败: {e}"
                result["error"] = error_msg
                logger.error(error_msg)
        else:
            result["error"] = "未找到可执行的代码"
            logger.warning("AI小车未找到可执行的代码")
        
        return result

