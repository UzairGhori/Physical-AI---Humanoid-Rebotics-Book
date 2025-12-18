import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from './styles.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Source[];
}

interface Source {
  title: string;
  path: string;
  snippet: string;
}

// Knowledge base for Physical AI & Humanoid Robotics
const knowledgeBase = [
  {
    topic: 'ROS 2',
    keywords: ['ros', 'ros2', 'ros 2', 'robot operating system', 'nodes', 'topics', 'services', 'actions'],
    content: `ROS 2 (Robot Operating System 2) is the foundation for modern robotics development. Key concepts include:

**Nodes**: Independent processes that perform computation
**Topics**: Named buses for asynchronous message passing
**Services**: Synchronous request/response communication
**Actions**: Long-running tasks with feedback

ROS 2 uses DDS (Data Distribution Service) for real-time, reliable communication. Popular distributions include Humble and Jazzy.`,
    source: { title: 'ROS 2 Fundamentals', path: '/docs/modules/ros2-fundamentals/intro', snippet: 'Introduction to ROS 2 architecture and core concepts' }
  },
  {
    topic: 'NVIDIA Isaac',
    keywords: ['isaac', 'nvidia', 'isaac sim', 'isaac ros', 'simulation', 'gpu', 'omniverse'],
    content: `NVIDIA Isaac is a comprehensive platform for robotics development:

**Isaac Sim**: High-fidelity robotics simulator built on Omniverse
**Isaac ROS**: GPU-accelerated ROS 2 packages for perception and navigation
**Isaac Lab**: Reinforcement learning environment for robot training

Isaac leverages NVIDIA GPUs for photorealistic rendering, physics simulation, and AI inference at scale.`,
    source: { title: 'NVIDIA Isaac Platform', path: '/docs/modules/nvidia-isaac/intro', snippet: 'NVIDIA Isaac simulation and deployment platform' }
  },
  {
    topic: 'Gazebo Simulation',
    keywords: ['gazebo', 'simulation', 'simulator', 'physics', 'ignition', 'gz'],
    content: `Gazebo is a powerful open-source robotics simulator:

**Physics Engines**: ODE, Bullet, DART, Simbody support
**Sensor Simulation**: Cameras, LiDAR, IMU, GPS, and more
**World Building**: Create complex environments with SDF format
**ROS Integration**: Seamless connection with ROS 2 via ros_gz

Gazebo Harmonic is the latest release with improved performance and features.`,
    source: { title: 'Gazebo Simulation', path: '/docs/modules/simulation/gazebo', snippet: 'High-fidelity robotics simulation with Gazebo' }
  },
  {
    topic: 'VLA Models',
    keywords: ['vla', 'vision language action', 'vision-language-action', 'pi0', 'openvla', 'rt-2', 'embodied ai', 'foundation model'],
    content: `Vision-Language-Action (VLA) models are the frontier of embodied AI:

**Architecture**: Combines vision encoders, language models, and action decoders
**Key Models**:
- RT-2: Google's robotics transformer
- OpenVLA: Open-source VLA from Stanford
- π0 (Pi-Zero): Physical Intelligence's generalist robot policy

VLA models enable robots to understand natural language instructions and visual context to generate precise actions.`,
    source: { title: 'VLA Models', path: '/docs/modules/vla-models/intro', snippet: 'Vision-Language-Action models for robot control' }
  },
  {
    topic: 'Humanoid Robotics',
    keywords: ['humanoid', 'bipedal', 'walking', 'manipulation', 'figure', 'tesla bot', 'optimus', 'atlas'],
    content: `Humanoid robotics represents the pinnacle of robotic systems:

**Key Challenges**:
- Bipedal locomotion and balance
- Dexterous manipulation
- Human-robot interaction
- Real-time planning and control

**Leading Platforms**: Boston Dynamics Atlas, Figure 01, Tesla Optimus, Unitree H1

Humanoids leverage VLA models for learning complex behaviors from demonstrations.`,
    source: { title: 'Humanoid Robotics', path: '/docs/modules/humanoid/intro', snippet: 'Building and programming humanoid robots' }
  },
  {
    topic: 'Course Overview',
    keywords: ['course', 'curriculum', 'modules', 'learning', 'syllabus', 'start', 'begin', 'overview'],
    content: `This comprehensive course covers four main modules:

**Module 1: ROS 2 Fundamentals** - Master the Robot Operating System
**Module 2: Simulation** - Gazebo and virtual environments
**Module 3: NVIDIA Isaac** - GPU-accelerated robotics
**Module 4: VLA Models** - Vision-Language-Action for embodied AI

Each module includes hands-on labs and practical projects. Start with the introduction to build a strong foundation.`,
    source: { title: 'Course Introduction', path: '/docs/intro', snippet: 'Complete course overview and learning path' }
  },
  {
    topic: 'URDF and Robot Models',
    keywords: ['urdf', 'xacro', 'robot model', 'robot description', 'joints', 'links', 'mesh'],
    content: `URDF (Unified Robot Description Format) defines robot structure:

**Links**: Rigid bodies with visual, collision, and inertial properties
**Joints**: Connections between links (revolute, prismatic, fixed, etc.)
**Xacro**: XML macro language for modular URDF files

Best practices:
- Use separate visual and collision meshes
- Define accurate inertial properties
- Leverage xacro for reusable components`,
    source: { title: 'Robot Modeling', path: '/docs/modules/ros2-fundamentals/urdf', snippet: 'Creating robot descriptions with URDF' }
  },
  {
    topic: 'Navigation',
    keywords: ['navigation', 'nav2', 'path planning', 'slam', 'mapping', 'localization', 'autonomous'],
    content: `Nav2 is the ROS 2 navigation stack for autonomous robots:

**Components**:
- Planner Server: Global path planning (NavFn, Smac)
- Controller Server: Local trajectory following (DWB, TEB)
- Recovery Server: Stuck detection and recovery behaviors
- BT Navigator: Behavior tree-based navigation orchestration

SLAM integration enables mapping while navigating in unknown environments.`,
    source: { title: 'Robot Navigation', path: '/docs/modules/ros2-fundamentals/navigation', snippet: 'Autonomous navigation with Nav2' }
  },
  {
    topic: 'Perception',
    keywords: ['perception', 'computer vision', 'camera', 'lidar', 'depth', 'point cloud', 'object detection'],
    content: `Robot perception enables understanding of the environment:

**Sensors**:
- RGB Cameras: Visual perception and object detection
- Depth Cameras: 3D scene understanding (RealSense, ZED)
- LiDAR: Precise distance measurements and mapping
- IMU: Orientation and motion tracking

Isaac ROS provides GPU-accelerated perception pipelines for real-time processing.`,
    source: { title: 'Robot Perception', path: '/docs/modules/nvidia-isaac/perception', snippet: 'Sensor integration and perception systems' }
  },
  {
    topic: 'Machine Learning',
    keywords: ['machine learning', 'ml', 'deep learning', 'neural network', 'training', 'inference', 'ai'],
    content: `Machine learning powers modern robotics:

**Applications**:
- Object detection and recognition
- Semantic segmentation
- Pose estimation
- Reinforcement learning for control

**Frameworks**: PyTorch, TensorFlow, Isaac Lab for RL
**Deployment**: TensorRT optimization, Isaac ROS inference nodes

VLA models represent the latest advancement combining vision, language, and action learning.`,
    source: { title: 'ML for Robotics', path: '/docs/modules/vla-models/ml-basics', snippet: 'Machine learning foundations for robotics' }
  }
];

function findRelevantContent(query: string): { response: string; sources: Source[] } {
  const queryLower = query.toLowerCase();
  const relevantTopics: typeof knowledgeBase = [];

  // Find matching topics based on keywords
  for (const topic of knowledgeBase) {
    const keywordMatch = topic.keywords.some(keyword => queryLower.includes(keyword));
    const topicMatch = queryLower.includes(topic.topic.toLowerCase());

    if (keywordMatch || topicMatch) {
      relevantTopics.push(topic);
    }
  }

  // If no specific match, check for general questions
  if (relevantTopics.length === 0) {
    const generalKeywords = ['what', 'how', 'explain', 'tell', 'help', 'learn', 'start'];
    const isGeneral = generalKeywords.some(kw => queryLower.includes(kw));

    if (isGeneral) {
      // Return course overview for general questions
      const overview = knowledgeBase.find(t => t.topic === 'Course Overview');
      if (overview) {
        relevantTopics.push(overview);
      }
    }
  }

  if (relevantTopics.length === 0) {
    return {
      response: `I don't have specific information about that topic in my knowledge base. However, I can help you with:

• **ROS 2** - Robot Operating System fundamentals
• **Gazebo** - Robotics simulation
• **NVIDIA Isaac** - GPU-accelerated robotics platform
• **VLA Models** - Vision-Language-Action for embodied AI
• **Humanoid Robotics** - Building humanoid systems
• **Navigation & Perception** - Autonomous robot capabilities

Feel free to ask about any of these topics!`,
      sources: []
    };
  }

  // Build response from relevant topics
  const responses = relevantTopics.slice(0, 2).map(t => t.content);
  const sources = relevantTopics.slice(0, 3).map(t => t.source);

  return {
    response: responses.join('\n\n---\n\n'),
    sources
  };
}

const suggestedQuestions = [
  'What is ROS 2?',
  'Explain VLA models',
  'How does NVIDIA Isaac work?',
  'Tell me about humanoid robots',
  'How do I start the course?',
  'What is Gazebo simulation?'
];

export default function RAGChatbot(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, scrollToBottom]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const generateId = () => Math.random().toString(36).substr(2, 9);

  const handleSendMessage = async (content: string) => {
    if (!content.trim()) return;

    const userMessage: Message = {
      id: generateId(),
      role: 'user',
      content: content.trim(),
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsTyping(true);

    // Simulate network delay for natural feel
    await new Promise(resolve => setTimeout(resolve, 800 + Math.random() * 700));

    const { response, sources } = findRelevantContent(content);

    const assistantMessage: Message = {
      id: generateId(),
      role: 'assistant',
      content: response,
      timestamp: new Date(),
      sources
    };

    setIsTyping(false);
    setMessages(prev => [...prev, assistantMessage]);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage(inputValue);
    }
  };

  const handleSuggestionClick = (question: string) => {
    handleSendMessage(question);
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* Chat Toggle Button */}
      <button
        className={`${styles.chatToggle} ${isOpen ? styles.chatToggleOpen : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open AI Assistant'}
      >
        {isOpen ? (
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M6 18L18 6M6 6l12 12" />
          </svg>
        ) : (
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
            <circle cx="12" cy="10" r="1" fill="currentColor" />
            <circle cx="8" cy="10" r="1" fill="currentColor" />
            <circle cx="16" cy="10" r="1" fill="currentColor" />
          </svg>
        )}
      </button>

      {/* Chat Window */}
      <div className={`${styles.chatWindow} ${isOpen ? styles.chatWindowOpen : ''}`}>
        {/* Header */}
        <div className={styles.chatHeader}>
          <div className={styles.chatHeaderInfo}>
            <div className={styles.chatAvatar}>
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                <path d="M12 2a2 2 0 0 1 2 2v1h3a2 2 0 0 1 2 2v2a4 4 0 0 1-4 4h-1v2h2a4 4 0 0 1 4 4v1a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2v-1a4 4 0 0 1 4-4h2v-2H9a4 4 0 0 1-4-4V7a2 2 0 0 1 2-2h3V4a2 2 0 0 1 2-2z" />
                <circle cx="9" cy="9" r="1" fill="currentColor" />
                <circle cx="15" cy="9" r="1" fill="currentColor" />
              </svg>
            </div>
            <div>
              <h3 className={styles.chatTitle}>Robotics AI Assistant</h3>
              <span className={styles.chatStatus}>
                <span className={styles.statusDot}></span>
                Online
              </span>
            </div>
          </div>
          <button className={styles.closeButton} onClick={toggleChat} aria-label="Close chat">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M6 18L18 6M6 6l12 12" />
            </svg>
          </button>
        </div>

        {/* Messages */}
        <div className={styles.chatMessages}>
          {messages.length === 0 ? (
            <div className={styles.welcomeMessage}>
              <div className={styles.welcomeIcon}>
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                  <path d="M12 2a2 2 0 0 1 2 2v1h3a2 2 0 0 1 2 2v2a4 4 0 0 1-4 4h-1v2h2a4 4 0 0 1 4 4v1a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2v-1a4 4 0 0 1 4-4h2v-2H9a4 4 0 0 1-4-4V7a2 2 0 0 1 2-2h3V4a2 2 0 0 1 2-2z" />
                </svg>
              </div>
              <h4>Welcome to the Robotics AI Assistant!</h4>
              <p>I can help you learn about ROS 2, NVIDIA Isaac, VLA models, and humanoid robotics. Ask me anything!</p>
              <div className={styles.suggestions}>
                <p className={styles.suggestionsLabel}>Try asking:</p>
                <div className={styles.suggestionButtons}>
                  {suggestedQuestions.map((question, index) => (
                    <button
                      key={index}
                      className={styles.suggestionButton}
                      onClick={() => handleSuggestionClick(question)}
                    >
                      {question}
                    </button>
                  ))}
                </div>
              </div>
            </div>
          ) : (
            <>
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`${styles.message} ${
                    message.role === 'user' ? styles.userMessage : styles.assistantMessage
                  }`}
                >
                  {message.role === 'assistant' && (
                    <div className={styles.messageAvatar}>
                      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                        <path d="M12 2a2 2 0 0 1 2 2v1h3a2 2 0 0 1 2 2v2a4 4 0 0 1-4 4h-1v2h2a4 4 0 0 1 4 4v1a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2v-1a4 4 0 0 1 4-4h2v-2H9a4 4 0 0 1-4-4V7a2 2 0 0 1 2-2h3V4a2 2 0 0 1 2-2z" />
                      </svg>
                    </div>
                  )}
                  <div className={styles.messageContent}>
                    <div className={styles.messageText}>
                      {message.content.split('\n').map((line, i) => (
                        <React.Fragment key={i}>
                          {line.startsWith('**') && line.endsWith('**') ? (
                            <strong>{line.slice(2, -2)}</strong>
                          ) : line.startsWith('- ') ? (
                            <span className={styles.listItem}>{line.slice(2)}</span>
                          ) : line.startsWith('• ') ? (
                            <span className={styles.listItem}>{line.slice(2)}</span>
                          ) : (
                            line
                          )}
                          {i < message.content.split('\n').length - 1 && <br />}
                        </React.Fragment>
                      ))}
                    </div>
                    {message.sources && message.sources.length > 0 && (
                      <div className={styles.sources}>
                        <span className={styles.sourcesLabel}>Sources:</span>
                        {message.sources.map((source, index) => (
                          <a
                            key={index}
                            href={source.path}
                            className={styles.sourceLink}
                          >
                            {source.title}
                          </a>
                        ))}
                      </div>
                    )}
                  </div>
                </div>
              ))}
              {isTyping && (
                <div className={`${styles.message} ${styles.assistantMessage}`}>
                  <div className={styles.messageAvatar}>
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                      <path d="M12 2a2 2 0 0 1 2 2v1h3a2 2 0 0 1 2 2v2a4 4 0 0 1-4 4h-1v2h2a4 4 0 0 1 4 4v1a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2v-1a4 4 0 0 1 4-4h2v-2H9a4 4 0 0 1-4-4V7a2 2 0 0 1 2-2h3V4a2 2 0 0 1 2-2z" />
                    </svg>
                  </div>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              )}
            </>
          )}
          <div ref={messagesEndRef} />
        </div>

        {/* Input */}
        <div className={styles.chatInput}>
          <input
            ref={inputRef}
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask about robotics, ROS 2, Isaac..."
            className={styles.input}
          />
          <button
            onClick={() => handleSendMessage(inputValue)}
            disabled={!inputValue.trim() || isTyping}
            className={styles.sendButton}
            aria-label="Send message"
          >
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M22 2L11 13M22 2l-7 20-4-9-9-4 20-7z" />
            </svg>
          </button>
        </div>
      </div>
    </>
  );
}
