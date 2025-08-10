import time
import torch
#import wandb


""" Trainers must implement train(), as consistant with dispatcher """ 

class PPO_Trainer:

    def __init__(self):
        pass
        
    def train(self, policy, checkpoint, replay_buffer, nodes, len_episode=500, t_iterations=0, iterations=100):
        state_former, performer, rewarder, command = nodes
        # Load policy checkpoint
        if checkpoint is not None:
            print(f"Loading checkpoint from: {checkpoint}")
            state_dict = torch.load(checkpoint)
            policy.load_state_dict(state_dict)
            print("Checkpoint loaded successfully.")
        
        if replay_buffer:
            ### Consume Replay Buffer ###
            while len(replay_buffer) > 0:
                replay = replay_buffer.pop(random.randrange(len(replay_buffer)))
                policy.compute_returns(replay)
            #
        
        len_episode = 500
        
        for t in range(0, t_iterations):
            performer.halt_performance()
            command.reset_learner()
            future = None
            done = False
            buffer = []
            state = state_former.get_state('/player/image_raw')
            for step in range(0, len_episode):
                action = kbi.demonstrate() # no logprobs or value head...
                future = performer.perform(action)
                rclpy.spin_once(performer, timeout_sec=0.01)
                if future and not future.done():
                    rclpy.spin_until_future_complete(performer, future)
                next_state = state_former.get_state('/player/image_raw')
                reward = rewarder.give()
                if reward > 0:
                    print("Hurray! (Giving server some time)")
                    time.sleep(1)
                    done = True
                #buffer.append((state, action, logp.item(), val.item(), reward, done))
            #buffer_replay.append(buffer)   
                if done:
                    break
        if replay_buffer:
            ### Consume Replay Buffer Again ###
            while len(replay_buffer) > 0:
                replay = replay_buffer.pop(random.randrange(len(replay_buffer)))
                policy.compute_returns(replay)
            #
               
        episode_rewards = []
        
        for i in range(0, iterations):
            performer.halt_performance()
            command.reset_learner()
            
            future = None
            done = False
            cumulative_reward = 0
            #############
            #  ROLLOUT  #
            #############
            buffer = []
            state = state_former.get_state('/player/image_raw')

            for step in range(0, len_episode):
                print(f'Step: {step}', end='\r', flush=True)
                with torch.no_grad():
                    act_idx, logp, val = policy.act(state)
                
                #future = node.perform(action)
                future = performer.perform(one_hot(act_idx.item()))
                rclpy.spin_once(node, timeout_sec=0.01)
                if future and not future.done():
                    rclpy.spin_until_future_complete(node, future)
                
                               
                next_state = state_former.get_state('/player/image_raw')
                reward = rewarder.give()
                cumulative_reward += reward
                if reward > 0:
                    print("Hurray! (Giving server some time)")
                    time.sleep(1)
                    rewarder._latest = 0 # hopeful overridew
                    done = True
                
                buffer.append((state, act_idx.item(), logp.item(), val.item(), reward, done))
                
                if (i + 1) % 3 == 0:
                    os.makedirs('/ws/custom/checkpoints', exist_ok=True)
                    torch.save(policy.state_dict(), f'/ws/custom/checkpoints/policy_iter_{i+1}.pth')
                    plot_topics = [episode_rewards]
                    plot(plot_topics)
                
                state = next_state
                if done:
                    break
                  
            node.halt_performance()
            #  END ROLLOUT
            
            ###################
            # COMPUTE RETURNS #
            ###################
            
            policy.compute_returns(buffer)
            
            episode_rewards.append(cumulative_reward)
            
class SAC_Trainer:
    def __init__(self, obs_dim, act_dim=6, gamma=0.99, tau=0.005, lr=3e-4, buffer_size=100000):
        self.gamma = gamma
        self.tau = tau
        self.policy = Policy(obs_dim, act_dim).to(device)

        self.actor_optimizer = optim.Adam(self.policy.actor.parameters(), lr=lr)
        self.q1_optimizer = optim.Adam(self.policy.q1.parameters(), lr=lr)
        self.q2_optimizer = optim.Adam(self.policy.q2.parameters(), lr=lr)
        self.alpha_optimizer = optim.Adam([self.policy.log_alpha], lr=lr)

        # Copy weights to target networks
        self.policy.target_q1.load_state_dict(self.policy.q1.state_dict())
        self.policy.target_q2.load_state_dict(self.policy.q2.state_dict())

        # Experience replay
        self.buffer = deque(maxlen=buffer_size)

        # Entropy target
        self.target_entropy = -act_dim

    def store_transition(self, transition):
        self.buffer.append(transition)

    def sample_batch(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        obs, act, rew, next_obs, done = map(np.array, zip(*batch))
        return (
            torch.tensor(obs, dtype=torch.float32),
            torch.tensor(act, dtype=torch.float32),
            torch.tensor(rew, dtype=torch.float32).unsqueeze(1),
            torch.tensor(next_obs, dtype=torch.float32),
            torch.tensor(done, dtype=torch.float32, device=self.device).unsqueeze(1)
        )

    def update(self, batch_size):
        if len(self.buffer) < batch_size:
            return

        obs, act, rew, next_obs, done = self.sample_batch(batch_size)

        with torch.no_grad():
            next_a, next_logp = self.policy.actor.sample(next_obs)
            target_q1 = self.policy.target_q1(next_obs, next_a)
            target_q2 = self.policy.target_q2(next_obs, next_a)
            target_q = torch.min(target_q1, target_q2) - self.policy.alpha * next_logp
            target_v = rew + (1 - done) * self.gamma * target_q

        # Q1, Q2 losses
        q1_loss = ((self.policy.q1(obs, act) - target_v) ** 2).mean()
        q2_loss = ((self.policy.q2(obs, act) - target_v) ** 2).mean()

        self.q1_optimizer.zero_grad()
        q1_loss.backward()
        self.q1_optimizer.step()

        self.q2_optimizer.zero_grad()
        q2_loss.backward()
        self.q2_optimizer.step()

        # Actor loss
        new_a, logp = self.policy.actor.sample(obs)
        q1_new = self.policy.q1(obs, new_a)
        q2_new = self.policy.q2(obs, new_a)
        q_new = torch.min(q1_new, q2_new)
        actor_loss = (self.policy.alpha * logp - q_new).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Alpha loss
        alpha_loss = -(self.policy.log_alpha * (logp + self.target_entropy).detach()).mean()
        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()
        self.policy.alpha = self.policy.log_alpha.exp()

        # Soft target update
        for target_param, param in zip(self.policy.target_q1.parameters(), self.policy.q1.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
        for target_param, param in zip(self.policy.target_q2.parameters(), self.policy.q2.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

        # WandB logging
        """
        wandb.log({
            "loss/q1": q1_loss.item(),
            "loss/q2": q2_loss.item(),
            "loss/actor": actor_loss.item(),
            "loss/alpha": alpha_loss.item(),
            "alpha": self.policy.alpha.item()
        })
        """

    def train(self, env, episodes=1000, batch_size=256):
        for ep in range(episodes):
            obs = env.reset()
            ep_reward = 0
            done = False
            while not done:
                action = self.policy.act(obs)
                next_obs, reward, done, _ = env.step(action)
                self.store_transition((obs, action, reward, next_obs, done))
                obs = next_obs
                ep_reward += reward
                self.update(batch_size)

            #wandb.log({"episode_reward": ep_reward})
