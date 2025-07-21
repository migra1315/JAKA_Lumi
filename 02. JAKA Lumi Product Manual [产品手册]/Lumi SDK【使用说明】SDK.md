# Lumi SDK说明
Lumi SDK分为以下三个部分：
* 手臂控制
* 升降与头部控制
* AGV控制

## 手臂控制
采用标准JAKA SDK[下载链接](https://www.jaka.com/prod-api/common/download/resource?resource=%2Fprofile%2Fupload%2F2025%2F04%2F25%2F20250425134342A024.zip)

版本要求>=v2.2.7

## AGV控制
通过TCP发送字符串指令来控制或获取AGV的运动。具体文档参见**AGV API手册.pdf**

python示例可参考 **04. JAKA Lumi Demo Case [示例]/agv** 目录下的资料


## 升降与头部电机控制
> 以下未做特殊说明时单位统一为度、毫米、千克、秒

采用http接口的方式控制与访问。总计有4个关节
* 关节1：升降，运动范围[0,300]mm
* 关节2：腰部旋转，运动范围[-140,140]度
* 关节3：头部旋转，运动范围[-180,180]度
* 关节4：头部俯仰，运动范围[-5,35]度
  
### web ui
浏览器访问http://192.168.10.90:5000，即可访问。

注意:当前仅提供基本状态监控等功能，后续将逐步完善

### 接口说明
> 以下的URL地址端口是固定的，IP地址需根据实际网络连接情况修改。采用wifi连接的情况下默认为192.168.10.90

#### 使能控制

POST: http://192.168.10.90:5000/api/extaxis/enable

Body段JSON参数：`{"enable":0}`

|参数名称|含义|
|---|---|
|enable|0：关闭使能；1：开启使能|

**python调用例子**：
```python
import requests
import json
response = requests.post("http://192.168.10.90:5000/api/extaxis/enable", json={"enable": 1})
if response.status_code != 200:
    print(f"Error: {response.status_code}")
    return
print("enable ok")
```
**curl调用**
```shell
curl -X POST http://192.168.10.90:5000/api/extaxis/enable \
     -H "Content-Type: application/json" \
     -d '{"enable": 1}'
```
其他语言可调用标准http库进行处理

#### 复位

POST: http://192.168.10.90:5000/api/extaxis/reset

Body段JSON参数：无

**python调用例子**：
```python
import requests
import json
response = requests.post("http://192.168.10.90:5000/api/extaxis/reset", json={})
if response.status_code != 200:
    print(f"Error: {response.status_code}")
    return
print("reset ok")
```

#### 运动到指定位置

POST: http://192.168.10.90:5000/api/extaxis/moveto

Body段JSON参数：{"pos":[0,0,0,0],"vel":100,"acc":100}

|参数名称|含义|
|---|---|
|pos|给定位置，分别为升降位置,腰部旋转，头部旋转，头部俯仰，单位为mm和度，超出运动范围的给定位置将报错|
|vel|速度比率,范围[0.1,100],超出范围将自动约束|
|acc|加速度比率,范围[1,100],超出范围将自动约束|

注意：
1. 调用将阻塞直到运动完成或错误
2. 调用前需要保证处于使能状态
3. 超出运动范围的给定位置将报错

**python调用例子**：
```python
import requests
import json
response = requests.post(
    "http://192.168.10.90:5000/api/extaxis/moveto",
    json={"pos": [200, 10, 10, 10], "vel": 100, "acc": 100},
)
if response.status_code != 200:
    print(f"Error: {response.status_code}")
    return
print("moveto success")
```

#### 获取状态

GET: http://192.168.10.90:5000/api/extaxis/status

Body段JSON参数：无

返回值：以数组的形式返回每个关节的状态，包含如下信息
|参数名称|含义|
|---|---|
|id|关节编号|
|pos|关节当前位置|
|vel|关节当前速度|
|toq|关节当前转矩|
|enable|关节使能状态位|
|error|关节错误标志位|
|ecode|关节错误码|

**python调用例子**：

```python
response = requests.get("http://192.168.10.90:5000/api/extaxis/status",)
if response.status_code != 200:
    print(f"Error: {response.status_code}")
    return
parsed_text_json = json.loads(response.text)  # Parse the text as JSON
print(f"JAKA Lumi state: {parsed_text_json[0]['pos']},{parsed_text_json[1]['pos']},{parsed_text_json[2]['pos']}")
```

### 综合例程

```python
import requests
import json
import time

API_URL = "http://localhost:5000/api/extaxis"

LUMI_ENABLE_URL = API_URL + "/enable"
LUMI_RESET_URL = API_URL + "/reset"
LUMI_MOVETO_URL = API_URL + "/moveto"
LUMI_STATUS_URL = API_URL + "/status"
LUMI_SYSINFO_URL = API_URL + "/sysinfo"
LUMI_GETSTATE_URL = API_URL + "/status"

def test():
    # get system infomation
    response = requests.get(LUMI_SYSINFO_URL)
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return
    parsed_text_json = json.loads(response.text)
    print(f"JAKA Lumi info: {parsed_text_json}")

    # reset all joint
    response = requests.post(LUMI_RESET_URL, json={})
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return
    print("reset ok")

    # enable all joint
    response = requests.post(LUMI_ENABLE_URL, json={"enable": 1})
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return
    print("enable ok")

    # read position
    response = requests.get(LUMI_GETSTATE_URL)
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return
    parsed_text_json = json.loads(response.text)  # Parse the text as JSON
    print(
        f"JAKA Lumi state: {parsed_text_json[0]['pos']},{parsed_text_json[1]['pos']},{parsed_text_json[2]['pos']}"
    )
    parsed_text_json[0]["pos"]

    # home
    response = requests.post(
        LUMI_MOVETO_URL,
        json={"pos": [0.0, 0, 0, 0], "vel": 100, "acc": 100},
    )
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return

    # move to position A blockly
    for i in range(100):
        response = requests.post(
            LUMI_MOVETO_URL,
            json={"pos": [0.0, -100.0, -90.0, -5.0], "vel": 100, "acc": 100},
        )
        if response.status_code != 200:
            print(f"Error: {response.status_code}")
            return
        print("move to A ok")

        # move to position B blockly
        response = requests.post(
            LUMI_MOVETO_URL,
            json={"pos": [200.0, 100.0, 90.0, 30.0], "vel": 100, "acc": 100},
        )
        if response.status_code != 200:
            print(f"Error: {response.status_code}")
            return
        print("move to B ok")

    # disable all joint
    time.sleep(2)
    response = requests.post(LUMI_ENABLE_URL, json={"enable": 0})
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return
    print("disable ok")


if __name__ == "__main__":
    test()

```
