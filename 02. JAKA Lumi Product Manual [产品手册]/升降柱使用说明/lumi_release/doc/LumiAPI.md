# Lumi API说明
LumiAPI分为以下三个部分：
* 手臂控制
* 升降与头部电机控制
* AGV控制

## 手臂控制
采用标准JAKA SDK[下载链接](https://www.jaka.com/prod-api/common/download/resource?resource=%2Fprofile%2Fupload%2F2025%2F04%2F25%2F20250425134342A024.zip)

版本要求>=v2.2.7

## AGV控制
通过TCP发送字符串指令来控制或获取AGV的运动。具体可参见**AGV API手册.pdf**


## 升降与头部电机控制
> 以下未做特殊说明时单位统一为度、毫米、千克、秒

采用http接口的方式控制与访问。总计有4个关节
* 关节1：升降，运动范围[0,300]mm
* 关节2：腰部旋转，运动范围[-140,140]度
* 关节3：头部旋转，运动范围[-180,180]度
* 关节4：头部俯仰，运动范围[-5,35]度
  
### web ui
浏览器访问http://10.5.5.100:5000，即可访问。
注意:当前仅提供基本状态监控等功能，后续将逐步完善

### 接口说明
> 以下的URL地址端口是固定的，IP地址需根据实际网络连接情况修改。采用wifi连接的情况下默认为10.5.5.100
#### 使能控制
POST: http://10.5.5.100:5000/api/extaxis/enable
Body段JSON参数：`{"enable":0}`
|参数名称|含义|
|---|---|
|enable|0：关闭使能；1：开启使能|

**python调用例子**：
```python
import requests
import json
response = requests.post("http://localhost:5000/api/extaxis/enable", json={"enable": 1})
if response.status_code != 200:
    print(f"Error: {response.status_code}")
    return
print("enable ok")
```
**curl调用**
```shell
curl -X POST http://localhost:5000/api/extaxis/enable \
     -H "Content-Type: application/json" \
     -d '{"enable": 1}'
```
其他语言可调用标准http库进行处理

#### 复位
#### 运动到指定位置
#### 获取状态
#### 获取版本

### 综合例程
```python
import lumi_url
import requests
import json
import time


def test():
    # get system infomation
    response = requests.get(lumi_url.LUMI_SYSINFO_URL)
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return
    parsed_text_json = json.loads(response.text)
    print(f"JAKA Lumi info: {parsed_text_json}")

    # reset all joint
    response = requests.post(lumi_url.LUMI_RESET_URL, json={})
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return
    print("reset ok")

    # enable all joint
    response = requests.post(lumi_url.LUMI_ENABLE_URL, json={"enable": 1})
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return
    print("enable ok")

    # read position
    response = requests.get(lumi_url.LUMI_GETSTATE_URL)
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
        lumi_url.LUMI_MOVETO_URL,
        json={"pos": [0.0, 0, 0, 0], "vel": 100, "acc": 100},
    )
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return

    # move to position A blockly
    for i in range(100):
        response = requests.post(
            lumi_url.LUMI_MOVETO_URL,
            json={"pos": [0.0, -100.0, -90.0, -5.0], "vel": 100, "acc": 100},
        )
        if response.status_code != 200:
            print(f"Error: {response.status_code}")
            return
        print("move to A ok")

        # move to position B blockly
        response = requests.post(
            lumi_url.LUMI_MOVETO_URL,
            json={"pos": [200.0, 100.0, 90.0, 30.0], "vel": 100, "acc": 100},
        )
        if response.status_code != 200:
            print(f"Error: {response.status_code}")
            return
        print("move to B ok")

    # disable all joint
    time.sleep(2)
    response = requests.post(lumi_url.LUMI_ENABLE_URL, json={"enable": 0})
    if response.status_code != 200:
        print(f"Error: {response.status_code}")
        return
    print("disable ok")


if __name__ == "__main__":
    test()

```