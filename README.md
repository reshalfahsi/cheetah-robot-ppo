# Teaching a Cheetah Robot to Run: Solving Continuous Control in Simulated Locomotion with Proximal Policy Optimization

 <div align="center">
    <a href="https://colab.research.google.com/github/reshalfahsi/cheetah-robot-ppo/blob/master/Cheetah_Robot_PPO.ipynb"><img src="https://colab.research.google.com/assets/colab-badge.svg" alt="colab"></a>
    <br />
 </div>



This project tackles the challenge of training a simulated Cheetah robot to run efficiently using reinforcement learning, focusing on the complexities of continuous action control. This project employs Proximal Policy Optimization (PPO), a stable and effective policy gradient algorithm, implemented with PyTorch, MuJoCo for realistic physics simulation, and Gymnasium as the environment framework. The policy and value networks, structured as multi-layer perceptrons, model actions via a Gaussian distribution, refined through iterative training with a clipped objective and Generalized Advantage Estimation (GAE) for stability. Key hyperparameters are tuned to optimize performance, and a deterministic policy—using the mean action—is adopted during evaluation to ensure consistency in the deterministic MuJoCo setting. The result is a Cheetah robot that achieves stable, agile locomotion with rising rewards, offering insights into continuous control and potential applications for real-world robotics.


## Experiment


Explore [this notebook](https://github.com/reshalfahsi/cheetah-robot-ppo/blob/master/Cheetah_Robot_PPO.ipynb) to watch the Cheetah robot in action and understand how PPO works controlling the robot.



## Result

## Reward Curve


<p align="center"> <img src="https://github.com/reshalfahsi/cheetah-robot-ppo/blob/master/assets/reward_curve.png" alt="reward_curve" > <br /> The reward tends to climb up in the course of the Cheetah robot training. </p>


## Qualitative Result


Here, the Cheetah robot is assigned to speedily maneuver forward as distantly as possible without stumbling and slipping, which can hinder its brisk gait mobility. It is illustrated in the ensuing GIF.


<p align="center">
    <a href="https://youtu.be/WxhuY_5ECvI"> 
    <img src="https://github.com/reshalfahsi/cheetah-robot-ppo/blob/master/assets/qualitative_cheetah_robot.gif" alt="qualitative_cheetah_robot" width="400"> 
    </a>
    <br />
    The Cheetah robot sprints predominantly stable in the simulated environment following the learned deterministic policy of PPO. 
</p>


## Credit

- [A Cat-Like Robot Real-Time Learning to Run](http://staff.elka.pw.edu.pl/~pwawrzyn/pub-s/0812_LSCLRR.pdf)
- [Proximal Policy Optimization Algorithms](https://arxiv.org/pdf/1707.06347)
- [Proximal Policy Optimization](https://spinningup.openai.com/en/latest/algorithms/ppo.html)
- [[gymnasium / mujoco] colaboratory で "HalfCheetah-v5" を動かそうとして FatalError: gladLoadGL error が発生したときの対策](https://qiita.com/siruku6/items/7545751b6f4d240427f6)
- [Proximal Policy Optimization](https://keras.io/examples/rl/ppo_cartpole/)
- [Half Cheetah](https://gymnasium.farama.org/environments/mujoco/half_cheetah/)
- [PyTorch Lightning](https://lightning.ai/docs/pytorch/latest/)
