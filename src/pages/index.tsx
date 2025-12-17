import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";

const HomePage: React.FC = () => {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Master humanoid robots with ROS2, Gazebo, NVIDIA Isaac, and embodied AI"
    >
      {/* HERO SECTION */}
      <header
        style={
          {
            minHeight: "100vh",
            background:
              "linear-gradient(135deg, #0A0A1A 0%, #1A0B2E 100%), url('https://images.unsplash.com/photo-1635070041078-e363dbe005cb?q=80&w=2070') center/cover no-repeat",
            display: "flex",
            flexDirection: "column",
            alignItems: "center",
            justifyContent: "center",
            textAlign: "center",
            color: "white",
            padding: "0 20px",
          } as React.CSSProperties
        }
      >
        <h2
          style={
            {
              fontSize: "4.5rem",
              fontWeight: 900,
              background: "linear-gradient(90deg, #00D4FF, #9D00FF)",
              WebkitBackgroundClip: "text",
              WebkitTextFillColor: "transparent",
              margin: "0 0 20px 0",
            } as React.CSSProperties
          }
        >
          Physical AI &amp; Humanoid Robotics
        </h2>

        <p
          style={
            {
              fontSize: "1.6rem",
              maxWidth: "900px",
              marginBottom: "50px",
              opacity: 0.9,
            } as React.CSSProperties
          }
        >
          Learn to design, simulate, and deploy real humanoid robots using ROS2,
          Gazebo, NVIDIA Isaac, and cutting-edge embodied AI techniques
        </p>

        <Link
          to="/docs/intro"
          style={
            {
              background: "linear-gradient(45deg, #00D4FF, #9D00FF)",
              color: "white",
              padding: "18px 60px",
              borderRadius: "50px",
              fontSize: "1.5rem",
              fontWeight: 700,
              textDecoration: "none",
              boxShadow: "0 10px 40px rgba(0,212,255,0.5)",
              transition: "all 0.3s",
              display: "inline-block",
            } as React.CSSProperties
          }
          onMouseEnter={(e) =>
            (e.currentTarget.style.transform = "translateY(-5px)")
          }
          onMouseLeave={(e) =>
            (e.currentTarget.style.transform = "translateY(0)")
          }
        >
          Start Learning Now →
        </Link>
      </header>

      {/* OPTIONAL CARDS – chahiye toh rakho, nahi toh delete */}
      <main style={{ padding: "100px 20px", textAlign: "center" }}>
        <div
          style={
            {
              display: "grid",
              gridTemplateColumns: "repeat(auto-fit, minmax(300px, 1fr))",
              gap: "40px",
              maxWidth: "1200px",
              margin: "0 auto",
            } as React.CSSProperties
          }
        >
          {["Zero to Walking Robot", "Industry Tools", "500+ Page Book"].map(
            (title) => (
              <div
                key={title}
                style={
                  {
                    background: "rgba(30,30,60,0.6)",
                    padding: "30px",
                    borderRadius: "16px",
                    border: "1px solid rgba(0,212,255,0.3)",
                  } as React.CSSProperties
                }
              >
                <h2 style={{ color: "#00D4FF" }}>{title}</h2>
                <p>
                  {title === "Zero to Walking Robot" &&
                    "Full journey with real code"}
                  {title === "Industry Tools" && "ROS2 • Gazebo • Isaac Sim"}
                  {title === "500+ Page Book" &&
                    "Lifetime access + all source code"}
                </p>
              </div>
            ),
          )}
        </div>
      </main>
    </Layout>
  );
};

export default HomePage;
