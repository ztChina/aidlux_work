import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import requests
from ament_index_python.packages import get_package_share_directory
import os

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        #选择LLM大模型，启动
        self.llm_name = "Qwen2.5-0.5B-Instruct_1-Q8_gguf"
        result =  subprocess.run(["sudo","aidllm","list","api"],capture_output=True, text=True).stdout
        self.get_logger().info(f"系统内已配置以下llm：\n{result}")
        if f"{self.llm_name}" in result:
            subprocess.run(["sudo","aidllm","start","api","-m",f"{self.llm_name}"])
        else:
            self.get_logger().error(f"未找到{self.llm_name}模型，请先配置模型")
        #启动知识库服务
        self.rag_name = "Test4"
        result =  subprocess.run(["sudo","aidllm","list","rag"],capture_output=True, text=True).stdout
        self.get_logger().info(f"系统内已配置以下知识库：\n{result}")
        if f"{self.rag_name}" in result:
            subprocess.run(["sudo","aidllm","start","rag","-n",f"{self.rag_name}"])
        else:
            self.get_logger().error(f"未找到{self.rag_name}模型，请先配置模型")


        ####################大模型请求配置################
        self.api_url = "http://127.0.0.1:8888/v1/chat/completions"  # 模型json文件中是0.0.0.0:8888，可以监听所有接口的8888端口
        self.api_headers = {    #请求头
            "Content-Type": "application/json"
            # 如果你的服务需要 token，则加上 Authorization
            # "Authorization": "Bearer sk-xxx"
        }
        ####################知识库请求配置######################
        self.rag_url = "http://127.0.0.1:18111/query"
        
        #订阅\发布节点
        self.subscription = self.create_subscription(String, 'coordinator/result', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(String, 'llm/reply', 10)
        self.get_logger().info("LLM节点启动成功")


    def listener_callback(self, msg):
        input_text = msg.data
        if input_text :
            # 知识库查询
            data = {
                "text": input_text,
                "collection_name": self.rag_name, 
                "top_k": 5,                       # 返回最相关的 top k 个结果
                "score_threshold": 70         # 相似度阈值，控制结果过滤
            }

            try:
                print("正在向知识库查询...")
                response = requests.post(self.rag_url, json=data)
                if response.status_code == 200:
                    print("知识库查询结果:", response.json())
                else:
                    print(f"知识库请求失败，状态码: {response.status_code}")
                    print("知识库错误信息:\n", response.text)
            except Exception as e:
                print("知识库请求异常:", e)

            retrieved_texts = [item['text'] for item in response.json()['data']]
            context = '\n'.join(retrieved_texts)

            # 大模型查询
            question = input_text
            prompt = f"""你的名字叫小笨蛋，你是一个植物陪伴机器人。以下是从知识库中检索到的相关信息：
                    {context}
                    根据以上内容，请回答用户的问题：
                    {question}
                    """
            data = {
                "model": self.llm_name,
                "messages": [
                    {"role": "system", "content": "你的名字叫小笨蛋，你是一个植物陪伴机器人。"},
                    {"role": "user", "content": prompt}
                ],
                "temperature": 0.7
            }
            print("正在向大模型查询...")
            response = requests.post(self.api_url, headers=self.api_headers, json=data)

            if response.status_code == 200:
                reply_json = response.json()
                reply = reply_json["choices"][0]["message"]["content"]
                print("AI回答：", reply)
                #发布大模型输出
                reply_msg = String()
                reply_msg.data = reply
                self.publisher_.publish(reply_msg)
                self.get_logger().info(f"AI回答文本已发布")
            else:
                print("请求失败：", response.status_code, response.text)
                return
        else :
            return

    
    def destroy_node(self):
        subprocess.run(["sudo","aidllm","stop","api"]) #stop大模型
        subprocess.run(["sudo","aidllm","stop","rag"])  #stop知识库
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
