# 基于Matlab和ROS的移动机器人智能工程联合仿真

<p style="text-align: right">使用ROS Noetic的仿真环境(python env)和Matlab齐全的自动驾驶工具箱(toolbox)</p>

*****
🤖  🚗

------

## 实验简单说明

**实验在控制器和里程计模型的章节为自己编写代码，后面定位和建图为取得更好效果使用matlab算法，包含了自己编写可视化接口代码**





文件夹子目录下配套了同步演示的gazebo视频和matlab生成gif作为对比参考



实验使用叉车模型基于[my_ROS_mobile_robot开源项目](https://github.com/eborghi10/my_ROS_mobile_robot)。特此鸣谢




**更多信息请查阅pdf,为包含动图的pdf格式,请使用Adode或者pdfexchage等支持pdf gif预览的文件查看**

## 实验效果


### ICP配准



### Amcl蒙特卡洛定位
`python ros端模型`:


<video controls>
  <source src="https://private-user-images.githubusercontent.com/82481204/316372167-a687505f-ba6d-44c4-8c64-a4a8b99a3d45.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MTEzMzUzMDgsIm5iZiI6MTcxMTMzNTAwOCwicGF0aCI6Ii84MjQ4MTIwNC8zMTYzNzIxNjctYTY4NzUwNWYtYmE2ZC00NGM0LThjNjQtYTRhOGI5OWEzZDQ1Lm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNDAzMjUlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjQwMzI1VDAyNTAwOFomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTI3OTU1YWVlNmYxNWU5ZmZlYzU0NTlkOWU2M2E0ZDFmNTNkY2VkNTkzOTBmZjY0N2JmYWEyODI0Njc0MTI5YjYmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JmFjdG9yX2lkPTAma2V5X2lkPTAmcmVwb19pZD0wIn0.RSAimEA6TRFztFPRNc3pDJjZA1La2h5vavR8a4QlHmI" type="video/mp4">
</video>


`matlab动图`：

<img src="https://github.com/BreezeConfirmingWms/nkuai_IntelligentProjectWork/blob/develop/AMCL%E8%92%99%E7%89%B9%E5%8D%A1%E6%B4%9B%E5%AE%9A%E4%BD%8D%E6%A8%A1%E5%9D%97/%E6%96%9C%E5%90%91%E5%8A%A8%E5%9B%BE/animationslanted.gif" alt="amcl定位+建模动图">

****


`By@YanMing,BreezeConfirmingWms@github.com，2023/5/31`




`git clone <url> --recursive`

or 

`git clone;git submoudle update --init --recursive`

👍:
