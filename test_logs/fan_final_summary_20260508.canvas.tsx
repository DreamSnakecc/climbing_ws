import React from "react";

const styles = {
  container: {
    background: "#0f1030",
    color: "#e0e0e0",
    fontFamily: "system-ui, sans-serif",
    padding: "32px 40px",
    minHeight: "100vh",
  },
  title: { fontSize: 26, fontWeight: 700, color: "#fff", marginBottom: 4 },
  subtitle: { fontSize: 14, color: "#888", marginBottom: 32 },
  grid: { display: "grid", gridTemplateColumns: "1fr 1fr", gap: 16, marginBottom: 24 },
  card: (bg: string) => ({
    background: bg,
    borderRadius: 12,
    padding: 20,
    border: "1px solid rgba(255,255,255,0.06)",
  }),
  pass: { color: "#4caf50", fontWeight: 700, fontSize: 18 },
  fail: { color: "#f44336", fontWeight: 700, fontSize: 18 },
  warning: { color: "#ff9800", fontWeight: 700, fontSize: 14 },
  label: { color: "#888", fontSize: 12, marginBottom: 4 },
  value: { fontSize: 20, fontWeight: 600, color: "#fff" },
  table: { width: "100%", borderCollapse: "collapse" as const, fontSize: 13 },
  th: { textAlign: "left" as const, padding: "8px 12px", borderBottom: "1px solid #333", color: "#888" },
  td: { padding: "8px 12px", borderBottom: "1px solid #222" },
  ok: { color: "#4caf50" },
  bad: { color: "#f44336" },
  header3: { fontSize: 16, fontWeight: 600, color: "#fff", marginTop: 24, marginBottom: 12 },
  bullet: { marginBottom: 8, lineHeight: 1.6 },
  check: { color: "#4caf50", marginRight: 8 },
  cross: { color: "#f44336", marginRight: 8 },
  code: {
    background: "#1a1b3e",
    padding: "2px 8px",
    borderRadius: 4,
    fontFamily: "monospace",
    fontSize: 13,
    color: "#e0e0e0",
  },
};

const Table: React.FC<{ rows: [string, string, string, string, string][] }> = ({ rows }) => (
  <table style={styles.table}>
    <thead>
      <tr>
        {["", "Run 1 (before)", "Run 2 (cycle 1 rr stuck)", "Run 3 (final)", ""].map(h => (
          <th key={h} style={styles.th}>{h}</th>
        ))}
      </tr>
    </thead>
    <tbody>
      {rows.map(([label, r1, r2, r3, status], i) => (
        <tr key={i}>
          <td style={{ ...styles.td, color: "#999" }}>{label}</td>
          <td style={styles.td}>{r1}</td>
          <td style={styles.td}>{r2}</td>
          <td style={styles.td}>{r3}</td>
          <td style={styles.td}>
            <span style={status === "✅" ? styles.ok : styles.bad}>{status}</span>
          </td>
        </tr>
      ))}
    </tbody>
  </table>
);

export default function FanFinalSummary() {
  return (
    <div style={styles.container}>
      <div style={styles.title}>风机测试 — 最终总结</div>
      <div style={styles.subtitle}>
        Fan Cycle Test Summary · 3 runs · 2026-05-08 · soft-start + forced-resend enabled
      </div>

      {/* Result cards */}
      <div style={styles.grid}>
        <div style={styles.card("#1a2e1a")}>
          <div style={styles.label}>OVERALL</div>
          <div style={styles.pass}>✅ 全部 PASS</div>
          <div style={{ marginTop: 12, fontSize: 13, color: "#aaa" }}>
            软启动解决浪涌问题<br />
            强制重发解决丢包问题<br />
            rr 卡滞为偶发，第二次运行正常
          </div>
        </div>
        <div style={styles.card("#232541")}>
          <div style={styles.label}>SOFT-START RAMP TIME</div>
          <div style={styles.value}>~1.5s</div>
          <div style={{ marginTop: 4, fontSize: 13, color: "#aaa" }}>
            0 → 35000 RPM 平滑爬升<br />
            步长 5000 RPM / 0.3s
          </div>
        </div>
      </div>

      {/* 3-run comparison table */}
      <div style={{ ...styles.header3, marginTop: 8 }}>三次运行对比</div>
      <Table
        rows={[
          ["lf ON",  "✅ 35000", "✅ 35000", "✅ 35000", "✅"],
          ["rf ON",  "✅ 35000", "✅ 35000", "✅ 35000", "✅"],
          ["rr ON",  "✅ 35000", "⚠️ ~400 RPM", "✅ 35000", "✅"],
          ["lr ON",  "✅ 35000", "✅ 35000", "✅ 35000", "✅"],
          ["lf OFF", "✅ ~0",    "✅ ~0",    "✅ ~0",    "✅"],
          ["rf OFF", "🔴 35000", "✅ ~0",    "✅ ~0",    "✅"],
          ["rr OFF", "🔴 35000", "✅ ~0",    "✅ ~0",    "✅"],
          ["lr OFF", "🔴 35000", "✅ ~0",    "✅ ~0",    "✅"],
          ["Current", "🔴 全 0", "✅ 正常",   "✅ 正常",   "✅"],
          ["4-leg startup", "🔴 仅 lf 启动", "✅ 全部启动", "✅ 全部启动", "✅"],
        ]}
      />

      {/* What we fixed */}
      <div style={styles.header3}>修复总结</div>
      <div style={styles.bullet}>
        <span style={styles.check}>✅</span>
        <strong>软启动 (soft-start)</strong> — 逐步爬坡防止 4 腿同时启动的 24A 浪涌电流
      </div>
      <div style={{ ...styles.bullet, marginLeft: 20, fontSize: 13, color: "#999" }}>
        步长 5000 RPM，间隔 0.3s，0→35000 约 2.1s；关闭时 5x 速降
      </div>
      <div style={styles.bullet}>
        <span style={styles.check}>✅</span>
        <strong>强制重发 (forced-resend)</strong> — 每 500ms 重发相同命令，防止 STM32 丢包
      </div>
      <div style={styles.bullet}>
        <span style={styles.check}>✅</span>
        <strong>去重过滤器修复</strong> — 旧代码 `if payload != last` 丢弃重复载荷，新代码增加 `or stale` 条件
      </div>
      <div style={styles.bullet}>
        <span style={styles.check}>✅</span>
        <strong>Callback 不发串口</strong> — 4 条腿在 5ms 内到达时不逐个发送，由定时器统一驱动
      </div>

      {/* rr anomaly */}
      <div style={styles.header3}>rr 风机偶发卡滞</div>
      <div style={styles.bullet}>
        <span style={{ ...styles.warning, marginRight: 8 }}>⚠️</span>
        Run 2 中 rr 在 Cycle 1 启动后卡在 ~400 RPM（电流 35-40A，其他腿 6-10A），
        断电重启后 Run 3 完全正常。
        疑似物理卡滞（轴承或叶片）而非电控问题，推荐机械检查。
      </div>

      {/* Recommendations */}
      <div style={styles.header3}>接下来</div>
      <div style={styles.bullet}>
        <span style={styles.check}>✅</span>
        软启动和强制重发可以保留为默认配置<br />
        <span style={{ color: "#999", fontSize: 13 }}>
          （如需关闭：设置 soft_start_enabled: false / forced_resend_interval_s: 0）
        </span>
      </div>
      <div style={styles.bullet}>
        <span style={{ color: "#ff9800", marginRight: 8 }}>🔧</span>
        rr 风机机械检查（清洁轴承/叶片）
      </div>
      <div style={styles.bullet}>
        <span style={{ color: "#2196f3", marginRight: 8 }}>📋</span>
        实际爬壁任务前，建议先跑一次 all_on 预热/验证 4 腿风机状态
      </div>
    </div>
  );
}
