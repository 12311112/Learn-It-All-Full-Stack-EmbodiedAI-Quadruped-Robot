# Learn-It-All-Full-Stack-EmbodiedAI-Quadruped-Robot
具身智能是了一个非常火的方向，但是，其入行门槛较高。我们团队设想通过极低的成本（1500人民币左右），让大家体验到完整的具身智能开发链路。完整链路涉及到机械设计，仿真环境搭建，算法（强化学习，IK控制器），以及部署。现在，我们完整开源了这个工作，让大家以极低成本体验到具身智能涉及全链路，学习到主流的仿真器、强化学习算法及部署gpa体验。整体项目非常适合具身智能行业入门者，大学生创新创业计划，毕设等方面。同时，为了供给大家学习所用，我们不是具身智能开发最小链路，我们还给大家跑通了IK控制器链路。我们采用isaaclab&mujuco两个主流的仿真器进行sim与sim2sim验证。
我们的主要流程如下图所示：
<img width="816" height="338" alt="image" src="https://github.com/user-attachments/assets/0eb33d81-4255-4275-a9ac-46993806f269" />
我们尽可能的让低成本，全体验这个理念生效，得到了如下的强化学习策略部署效果：
https://github.com/user-attachments/assets/0e937bb2-8c6b-4ac0-97f3-5103152a0ea5
https://github.com/user-attachments/assets/c23fae7b-19d8-4e9f-8c7c-3008be9b9506
同时，我们也验证了IK控制器在实机上的效果：
https://github.com/user-attachments/assets/35e0f9d1-ff54-483b-a3b3-77c9c0c3400d

我们的工作主要包括有：
1.在solidworks中完成机械设计本体及urdf文件转换，同时在 https://viewer.robotsfan.com/ 中查看urdf转出的正确性，也在这里我们给出物料清单。

2.在isaaclab中将urdf文件转化成usd文件，在isaaclab中进行马尔可夫训练环境设计，物理标定等并行开始训练。

3.在mujuco中将urdf文件转换成xml场景描述文件，并训练了locomotion任务，导出onnx文件。

4.在jetson nano中（可用树莓派代替，算力要求不高），进行了rl策略实机部署及IK控制器验证。






