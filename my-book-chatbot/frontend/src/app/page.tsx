"use client";

import { useState, useEffect, useRef } from "react";
import Image from "next/image";

export default function Home() {
  const [chatVisible, setChatVisible] = useState(false);
  const [userId, setUserId] = useState<string>("user_" + Date.now());
  const [selectedText, setSelectedText] = useState<string>("");
  const [showHighlightConfirm, setShowHighlightConfirm] = useState(false);
  const [highlightPosition, setHighlightPosition] = useState({ x: 0, y: 0 });
  const [darkMode, setDarkMode] = useState(false);
  const [messages, setMessages] = useState<
    { id: string; content: string; role: string }[]
  >([
    {
      id: "1",
      content:
        "Hello! I'm your AI Tutor assistant. How can I help you with the documentation?",
      role: "assistant",
    },
  ]);
  const [inputMessage, setInputMessage] = useState<string>("");
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const mainRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== "") {
        const selectedText = selection.toString();
        if (selectedText.length > 0) {
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();

          setSelectedText(selectedText);
          setHighlightPosition({ x: rect.right, y: rect.top });
          setShowHighlightConfirm(true);
        }
      } else {
        setShowHighlightConfirm(false);
      }
    };

    const handleClick = () => {
      if (!window.getSelection()?.toString().trim()) {
        setShowHighlightConfirm(false);
      }
    };

    document.addEventListener("mouseup", handleSelection);
    document.addEventListener("click", handleClick);

    return () => {
      document.removeEventListener("mouseup", handleSelection);
      document.removeEventListener("click", handleClick);
    };
  }, []);

  // Send message with selected text
  const sendSelectedTextMessage = async () => {
    if (!selectedText.trim()) return;

    try {
      // Close the confirmation and open the chat
      setShowHighlightConfirm(false);
      setChatVisible(true);

      // Add the selected text as a user message
      const userMessage = {
        id: Date.now().toString(),
        content: `Regarding this text: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? "..." : ""}" Can you explain this?`,
        role: "user",
      };

      setMessages((prev) => [...prev, userMessage]);
    } catch (error) {
      console.error("Error sending selected text:", error);
    }
  };

  // Send message to backend
  const sendMessage = async () => {
    if (!inputMessage.trim() || isLoading) return;

    setIsLoading(true);

    // Add user message to chat
    const userMessage = {
      id: Date.now().toString(),
      content: inputMessage,
      role: "user",
    };

    setMessages((prev) => [...prev, userMessage]);
    const newInput = inputMessage;
    setInputMessage("");

    try {
      // Call backend API
      const response = await fetch(`${process.env.NEXT_PUBLIC_BACKEND_URL}/chat/default_thread`, // Using a default thread ID
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({
            message: newInput,
            user_id: userId,
          }),
        },
      );

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: data.id,
          content: data.content,
          role: "assistant",
        };
        setMessages((prev) => [...prev, botMessage]);
      } else {
        const errorMessage = {
          id: Date.now().toString(),
          content: "Sorry, I encountered an error processing your request.",
          role: "assistant",
        };
        setMessages((prev) => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error("Error sending message:", error);
      const errorMessage = {
        id: Date.now().toString(),
        content: "Sorry, I'm having trouble connecting to the server.",
        role: "assistant",
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Toggle dark mode
  const toggleDarkMode = () => {
    setDarkMode(!darkMode);
  };

  // Handle Enter key press for message input
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div
      className={`min-h-screen font-sans ${darkMode ? "bg-gradient-to-br from-gray-900 via-blue-900 to-indigo-900" : "bg-gradient-to-br from-blue-50 via-indigo-50 to-purple-50"}`}
    >
      <main
        ref={mainRef}
        className={`flex min-h-screen flex-col ${darkMode ? "bg-gray-900" : "bg-white"}`}
      >
        {/* Header */}
        <header
          className={`sticky top-0 z-40 w-full backdrop-blur-md transition-all duration-300 ${
            darkMode
              ? "bg-gray-900/80 border-b border-gray-800"
              : "bg-white/80 border-b border-gray-200"
          }`}
        >
          <div className="container mx-auto px-4 py-4 flex justify-between items-center">
            <div className="flex items-center space-x-2">
              <div
                className={`w-10 h-10 rounded-lg flex items-center justify-center ${
                  darkMode ? "bg-blue-600" : "bg-blue-500"
                }`}
              >
                <span className="text-white font-bold text-xl">🤖</span>
              </div>
              <h1
                className={`text-xl font-bold ${darkMode ? "text-white" : "text-gray-900"}`}
              >
                AI Documentation Assistant
              </h1>
            </div>

            {/* Dark Mode Toggle */}
            <button
              onClick={toggleDarkMode}
              className={`p-2 rounded-full transition-all duration-300 ${
                darkMode
                  ? "bg-yellow-400 text-gray-900 hover:bg-yellow-300"
                  : "bg-gray-800 text-yellow-400 hover:bg-gray-700"
              }`}
              aria-label={
                darkMode ? "Switch to light mode" : "Switch to dark mode"
              }
            >
              {darkMode ? "☀️" : "🌙"}
            </button>
          </div>
        </header>

        {/* Hero Section */}
        <section className="flex-1 flex flex-col items-center justify-center px-4 py-12">
          <div className="max-w-4xl mx-auto text-center mb-12">
            <h2
              className={`text-4xl md:text-6xl font-bold mb-6 bg-clip-text text-transparent bg-gradient-to-r ${
                darkMode
                  ? "from-blue-400 to-purple-400"
                  : "from-blue-600 to-purple-600"
              }`}
            >
              Your AI-Powered Documentation Assistant
            </h2>
            <p
              className={`text-xl mb-8 ${darkMode ? "text-gray-300" : "text-gray-600"}`}
            >
              Get instant answers to your questions about robotics, AI, and
              documentation with our intelligent chatbot.
            </p>

            <div
              className={`inline-flex items-center space-x-2 px-6 py-3 rounded-full ${
                darkMode
                  ? "bg-gray-800 text-blue-300"
                  : "bg-blue-100 text-blue-700"
              }`}
            >
              <span className="animate-pulse">●</span>
              <span>AI Assistant Online</span>
            </div>
          </div>

          {/* Features Grid */}
          <div className="grid md:grid-cols-3 gap-8 mb-16 w-full max-w-6xl">
            {[
              {
                icon: "📖",
                title: "Documentation Q&A",
                desc: "Ask questions about your documentation and get instant answers",
              },
              {
                icon: "💡",
                title: "Smart Search",
                desc: "Find relevant information across your entire documentation",
              },
              {
                icon: "🎯",
                title: "Context Aware",
                desc: "Understands context and provides precise answers",
              },
            ].map((feature, index) => (
              <div
                key={index}
                className={`p-6 rounded-xl backdrop-blur-sm transition-all duration-300 hover:scale-105 ${
                  darkMode
                    ? "bg-gray-800/50 border border-gray-700 hover:bg-gray-800/70"
                    : "bg-white/50 border border-gray-200 hover:bg-white/70"
                } shadow-lg hover:shadow-xl`}
              >
                <div className="text-3xl mb-4">{feature.icon}</div>
                <h3
                  className={`text-lg font-semibold mb-2 ${darkMode ? "text-white" : "text-gray-900"}`}
                >
                  {feature.title}
                </h3>
                <p className={darkMode ? "text-gray-400" : "text-gray-600"}>
                  {feature.desc}
                </p>
              </div>
            ))}
          </div>
        </section>

        {/* Highlight Confirmation Popup */}
        {showHighlightConfirm && selectedText && (
          <div
            className={`fixed z-50 backdrop-blur-md rounded-xl shadow-2xl p-4 max-w-xs border ${
              darkMode
                ? "bg-gray-800/90 border-gray-700 text-white"
                : "bg-white/90 border-gray-200 text-gray-900"
            }`}
            style={{
              left: `${highlightPosition.x + 10}px`,
              top: `${highlightPosition.y - 60}px`,
              transform: "translateY(-100%)",
            }}
          >
            <p className="text-sm mb-3">
              <strong>You selected:</strong> "{selectedText.substring(0, 60)}
              {selectedText.length > 60 ? "..." : ""}"
            </p>
            <div className="flex space-x-2">
              <button
                onClick={sendSelectedTextMessage}
                className={`text-sm py-2 px-4 rounded-lg transition-colors ${
                  darkMode
                    ? "bg-blue-600 hover:bg-blue-700 text-white"
                    : "bg-blue-500 hover:bg-blue-600 text-white"
                }`}
              >
                Ask AI
              </button>
              <button
                onClick={() => setShowHighlightConfirm(false)}
                className={`text-sm py-2 px-4 rounded-lg transition-colors ${
                  darkMode
                    ? "bg-gray-700 hover:bg-gray-600 text-gray-200"
                    : "bg-gray-200 hover:bg-gray-300 text-gray-800"
                }`}
              >
                Cancel
              </button>
            </div>
          </div>
        )}

        {/* Floating Chat Button - Compact Docusaurus Theme */}
        <div className="fixed bottom-6 right-6 z-50">
          <button
            onClick={() => setChatVisible(!chatVisible)}
            className={`w-12 h-12 rounded-full shadow-2xl transition-all duration-300 transform hover:scale-110 flex items-center justify-center text-lg ${
              darkMode
                ? "bg-gradient-to-r from-[#00D4FF] to-[#9D00FF] text-white hover:shadow-lg hover:shadow-[#00D4FF]/50"
                : "bg-gradient-to-r from-[#00D4FF] to-[#9D00FF] text-white hover:shadow-lg hover:shadow-[#00D4FF]/50"
            }`}
            style={{ fontFamily: "'Orbitron', sans-serif" }}
          >
            💬
          </button>
        </div>

        {/* Custom Chat Panel - Only show when chat is visible - Compact Docusaurus Theme */}
        {chatVisible && (
          <div
            className={`fixed bottom-24 right-6 w-full max-w-sm h-[450px] z-50 rounded-2xl overflow-hidden shadow-2xl backdrop-blur-xl border flex flex-col ${
              darkMode
                ? "bg-gradient-to-br from-[#0A0A1A]/90 to-[#1A0B2E]/70 border-[#00D4FF]/20"
                : "bg-gradient-to-br from-white/90 to-gray-50/90 border-gray-200"
            }`}
            style={{ fontFamily: "'Exo 2', sans-serif" }}
          >
            {/* Chat Header with Docusaurus theme */}
            <div
              className={`relative p-3 flex-shrink-0 ${
                darkMode
                  ? "bg-gradient-to-r from-[#0A0A1A] to-[#1A0B2E] border-b border-[#00D4FF]/20"
                  : "bg-gradient-to-r from-[#00D4FF] to-[#9D00FF] border-b border-gray-200"
              } text-white overflow-hidden`}
            >
              <div className="absolute inset-0 bg-gradient-to-r from-[#00D4FF]/10 via-transparent to-[#9D00FF]/10 animate-pulse"></div>
              <div className="flex items-center justify-between relative z-10">
                <div className="flex items-center space-x-2">
                  <div className="relative">
                    <div className="w-2.5 h-2.5 bg-[#00D4FF] rounded-full animate-pulse shadow-lg shadow-[#00D4FF]/50"></div>
                  </div>
                  <div>
                    <h3
                      className="font-bold text-sm"
                      style={{ fontFamily: "'Orbitron', sans-serif" }}
                    >
                      🤖 AI Assistant
                    </h3>
                  </div>
                </div>
                <button
                  onClick={() => setChatVisible(false)}
                  className="w-7 h-7 rounded-full bg-white/20 backdrop-blur-sm hover:bg-white/30 transition-all duration-300 flex items-center justify-center text-white hover:scale-110"
                >
                  ✕
                </button>
              </div>
            </div>

            {/* Messages Container with Docusaurus theme */}
            <div
              className="flex-1 overflow-y-auto p-3 space-y-3 flex-grow"
              style={{
                minHeight: 0,
                scrollbarWidth: "thin",
                fontFamily: "'Exo 2', sans-serif",
              }}
            >
              {messages.map((msg) => (
                <div
                  key={msg.id}
                  className={`group relative max-w-[85%] transition-all duration-300 hover:scale-[1.02] ${
                    msg.role === "user" ? "ml-auto" : "mr-auto"
                  }`}
                >
                  <div
                    className={`p-3 rounded-xl shadow-lg backdrop-blur-sm border ${
                      msg.role === "user"
                        ? darkMode
                          ? "bg-gradient-to-r from-[#00D4FF]/20 to-[#00BFF3]/20 text-white rounded-br-md border-[#00D4FF]/30"
                          : "bg-gradient-to-r from-[#00D4FF] to-[#9D00FF] text-white rounded-br-md border-[#00D4FF]"
                        : darkMode
                          ? "bg-gradient-to-r from-[#0A0A1A]/50 to-[#1A0B2E]/50 text-gray-100 rounded-bl-md border-[#00D4FF]/20"
                          : "bg-gradient-to-r from-gray-100 to-gray-50 text-gray-800 rounded-bl-md border-gray-200"
                    }`}
                  >
                    <div className="flex items-start space-x-1.5">
                      {msg.role === "assistant" && (
                        <div
                          className={`w-5 h-5 rounded-full flex items-center justify-center text-xs ${
                            darkMode ? "bg-[#00D4FF]" : "bg-[#00D4FF]"
                          }`}
                        >
                          🤖
                        </div>
                      )}
                      <div className="flex-1">{msg.content}</div>
                      {msg.role === "user" && (
                        <div
                          className={`w-5 h-5 rounded-full flex items-center justify-center text-xs ${
                            darkMode ? "bg-blue-500" : "bg-blue-400"
                          }`}
                        >
                          👤
                        </div>
                      )}
                    </div>
                  </div>
                </div>
              ))}
              {isLoading && (
                <div
                  className={`group relative max-w-[85%] mr-auto transition-all duration-300`}
                >
                  <div
                    className={`p-3 rounded-xl shadow-lg backdrop-blur-sm border ${
                      darkMode
                        ? "bg-gradient-to-r from-[#0A0A1A]/50 to-[#1A0B2E]/50 text-gray-100 rounded-bl-md border-[#00D4FF]/20"
                        : "bg-gradient-to-r from-gray-100 to-gray-50 text-gray-800 rounded-bl-md border-gray-200"
                    }`}
                  >
                    <div className="flex items-center space-x-1.5">
                      <div
                        className={`w-5 h-5 rounded-full flex items-center justify-center text-xs ${
                          darkMode ? "bg-[#00D4FF]" : "bg-[#00D4FF]"
                        }`}
                      >
                        🤖
                      </div>
                      <div className="flex space-x-1">
                        <div className="w-1.5 h-1.5 rounded-full bg-[#00D4FF] animate-bounce"></div>
                        <div className="w-1.5 h-1.5 rounded-full bg-[#00D4FF] animate-bounce delay-100"></div>
                        <div className="w-1.5 h-1.5 rounded-full bg-[#00D4FF] animate-bounce delay-200"></div>
                      </div>
                    </div>
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>

            {/* Input Area with Docusaurus theme */}
            <div
              className={`p-3 border-t backdrop-blur-sm flex-shrink-0 ${
                darkMode
                  ? "border-[#00D4FF]/20 bg-[#0A0A1A]/50"
                  : "border-gray-200 bg-white/50"
              }`}
            >
              <div className="flex gap-1.5">
                <input
                  type="text"
                  value={inputMessage}
                  onChange={(e) => setInputMessage(e.target.value)}
                  onKeyPress={handleKeyPress}
                  placeholder="Ask..."
                  className={`flex-1 p-2 rounded-lg border backdrop-blur-sm ${
                    darkMode
                      ? "bg-[#0A0A1A]/50 border-[#00D4FF]/30 text-white placeholder-gray-400 text-sm"
                      : "bg-white/70 border-gray-300 text-gray-900 placeholder-gray-600 text-sm"
                  } transition-all duration-300 focus:ring-2 focus:ring-[#00D4FF]/50 focus:border-[#00D4FF]`}
                  disabled={isLoading}
                  style={{ fontFamily: "'Exo 2', sans-serif" }}
                />
                <button
                  onClick={sendMessage}
                  disabled={isLoading || !inputMessage.trim()}
                  className={`px-3 py-2 rounded-lg transition-all duration-300 text-sm ${
                    isLoading || !inputMessage.trim()
                      ? "bg-gray-300 text-gray-500 cursor-not-allowed"
                      : darkMode
                        ? "bg-gradient-to-r from-[#00D4FF] to-[#9D00FF] text-white hover:from-[#00BFF3] hover:to-[#8B00E0] hover:shadow-lg hover:shadow-[#00D4FF]/25"
                        : "bg-gradient-to-r from-[#00D4FF] to-[#9D00FF] text-white hover:from-[#00BFF3] hover:to-[#8B00E0] hover:shadow-lg hover:shadow-[#00D4FF]/25"
                  }`}
                  style={{ fontFamily: "'Exo 2', sans-serif" }}
                >
                  Send
                </button>
              </div>
            </div>
          </div>
        )}
      </main>
    </div>
  );
}
