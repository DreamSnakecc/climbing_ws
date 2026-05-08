import { Card, CardBody, CardHeader, Divider, H1, H2, Pill, Stack, Table, Text } from 'cursor/canvas';
import { useHostTheme } from 'cursor/canvas';

// ─── Severity label components ───────────────────────────────────────

function SeverityCritical() {
  return <Pill tone="deleted" active>CRITICAL</Pill>;
}

function SeverityWarning() {
  return <Pill tone="warning" active>WARNING</Pill>;
}

// ─── Stuck-value indicator for table cells ───────────────────────────

function StuckCell({ value }: { value: string }) {
  const { tokens: t } = useHostTheme();
  return (
    <span style={{ fontWeight: 600, color: t.accent.primary }}>
      {value} &#x2715;
    </span>
  );
}

const tableHeaders = ['Cycle', 'Phase', 'lf RPM', 'rf RPM', 'rr RPM', 'lr RPM'];

// ─── Component ──────────────────────────────────────────────────────

export default function FanCycleAnalysis() {
  const { tokens: t } = useHostTheme();

  return (
    <Stack gap={20}>

      {/* ── Title ────────────────────────────────────────────────── */}
      <Stack gap={4}>
        <H1 style={{ margin: 0 }}>
          {'\u98CE\u673A\u5FAA\u73AF\u6D4B\u8BD5\u8BCA\u65AD\u62A5\u544A'}
          <span style={{ fontWeight: 400, marginLeft: 8, color: t.text.secondary }}>
            (Fan Cycle Test Diagnostic Report)
          </span>
        </H1>
        <Text tone="secondary" size="small">
          test_logs/fan_cycle_20260508_174328.csv &middot; 1500 samples &middot; 3 cycles
        </Text>
      </Stack>

      {/* ── Section 1: Test Overview ─────────────────────────────── */}
      <Divider />

      <H2>
        1. {'\u6D4B\u8BD5\u6982\u89C8'}
        <span style={{ fontWeight: 400, marginLeft: 6, color: t.text.secondary }}>
          (Test Overview)
        </span>
      </H2>
      <Text>
        Each cycle follows the sequence:{' '}
        <Text weight="semibold" as="span">all_off</Text>
        {' '}(2s baseline) &rarr;{' '}
        <Text weight="semibold" as="span">all_on</Text>
        {' '}(5s at 35000 RPM) &rarr;{' '}
        <Text weight="semibold" as="span">seq_off</Text>
        {' '}(4s/leg x 4 legs: lf &rarr; rf &rarr; rr &rarr; lr) &rarr;{' '}
        <Text weight="semibold" as="span">all_off</Text>
        {' '}(2s settle). Total ~30s per cycle, 3 cycles repeating.
      </Text>

      {/* ── Section 2: Key Findings ──────────────────────────────── */}
      <Divider />

      <H2>
        2. {'\u5173\u952E\u53D1\u73B0'}
        <span style={{ fontWeight: 400, marginLeft: 6, color: t.text.secondary }}>
          (Key Findings)
        </span>
      </H2>

      {/* Finding 1 */}
      <Card>
        <CardHeader trailing={<SeverityCritical />}>
          {'rf \u98CE\u673A\u4E0D\u54CD\u5E94\u5173\u95ED\u6307\u4EE4'}
        </CardHeader>
        <CardBody>
          <Stack gap={8}>
            <Text weight="semibold" tone="secondary" size="small">
              rf fan unresponsive to OFF command
            </Text>
            <Text size="small">
              In all 3 cycles, seq_off step 2 commands rf to 0 RPM, but rf reads
              ~34998-35001 RPM throughout the entire step.
            </Text>
            <Text size="small">
              At the end of each cycle, all fans are commanded OFF. rf stays at
              35000 RPM while others (lf, lr) drop to ~0.
            </Text>
            <Text size="small">
              Cycle 2 all_off baseline: rf=35000 (still running from cycle 1).
            </Text>
            <Text size="small">
              Cycle 3 all_off baseline: rf=34997-35000.
            </Text>
            <Text weight="semibold" size="small" style={{ color: t.accent.primary }}>
              rf never turns off during the entire 78-second test.
            </Text>
          </Stack>
        </CardBody>
      </Card>

      {/* Finding 2 */}
      <Card>
        <CardHeader trailing={<SeverityCritical />}>
          {'rr \u548C lr \u5728 cycle 2-3 \u4E5F\u4E0D\u54CD\u5E94\u5173\u95ED\u6307\u4EE4'}
        </CardHeader>
        <CardBody>
          <Stack gap={8}>
            <Text weight="semibold" tone="secondary" size="small">
              rr and lr unresponsive in cycles 2-3
            </Text>
            <Text size="small">
              Cycle 1 seq_off step 3 (rr OFF): rr partially drops to ~0 but
              oscillates 163-753 RPM.
            </Text>
            <Text size="small">
              Cycle 1 seq_off step 4 (lr OFF): lr reads 35001-35004 (cmd=0)
              {' \u2014'} stuck ON.
            </Text>
            <Text size="small">
              Cycles 2-3 seq_off steps 3-4: both rr and lr read ~35000 when
              commanded to 0.
            </Text>
            <Text size="small">
              Cycle 2 all_off settle: rf=35000, rr=35000 (both stuck).
            </Text>
            <Text size="small">
              Cycle 3 all_off settle: rf=35000, rr=35000 (still stuck).
            </Text>
          </Stack>
        </CardBody>
      </Card>

      {/* Finding 3 */}
      <Card>
        <CardHeader trailing={<SeverityWarning />}>
          {'RPM \u8BFB\u53D6\u5F02\u5E38'}
        </CardHeader>
        <CardBody>
          <Stack gap={8}>
            <Text weight="semibold" tone="secondary" size="small">
              Anomalous RPM readings
            </Text>
            <Text size="small">
              Cycle 1 all_on: Only lf ramp up is visible. rf/rr/lr read ~0
              despite later proving they were running.
            </Text>
            <Text size="small">
              Cycle 1 seq_off step 1 (lf OFF): rf reads 28805, lr reads 28775
              {' \u2014'} they WERE running but RPM was not reported earlier.
            </Text>
            <Text size="small">
              Possible: RPM sensor reporting on some channels only starts after
              certain events.
            </Text>
          </Stack>
        </CardBody>
      </Card>

      {/* Finding 4 */}
      <Card>
        <CardHeader trailing={<SeverityWarning />}>
          {'\u98CE\u673A\u7535\u6D41\u8BFB\u6570\u5F02\u5E38'}
        </CardHeader>
        <CardBody>
          <Stack gap={8}>
            <Text weight="semibold" tone="secondary" size="small">
              Fan currents all near zero
            </Text>
            <Text size="small">
              All 4 fans show average current {'\u2264'} 7A even when running at
              35000 RPM. Most readings are 0.0000 A.
            </Text>
            <Text size="small">
              The fan_currents topic may not be published correctly.
            </Text>
          </Stack>
        </CardBody>
      </Card>

      {/* ── Section 3: Per-Cycle RPM Summary ─────────────────────── */}
      <Divider />

      <H2>
        3. {'\u9010\u5468\u671F RPM \u6458\u8981'}
        <span style={{ fontWeight: 400, marginLeft: 6, color: t.text.secondary }}>
          (Per-Cycle RPM Summary)
        </span>
      </H2>

      <Table
        headers={tableHeaders}
        rows={
          // table rows built inside component so StuckCell can use hooks
          [
            ['1', 'all_on (steady)', '~35000', '~0', '~0', '~0'],
            ['1', 'seq_off step1 (lf OFF)', '~6100 \u2193', '~28805', '~407', '~28775'],
            ['1', 'seq_off step2 (rf OFF)', '~28892', <StuckCell value="~35000" />, '~340', '~35000'],
            ['1', 'seq_off step3 (rr OFF)', '~34998', '~35000', '~428 \u2193', '~35001'],
            ['1', 'seq_off step4 (lr OFF)', '~35000', '~35000', '~368', <StuckCell value="~35001" />],
            ['1', 'all_off settle', '~0', <StuckCell value="35000" />, '~300', '~0'],

            ['2', 'all_on (steady)', '~35000', '~0', '~0', '~0'],
            ['2', 'seq_off step1 (lf OFF)', '~6100 \u2193', '~28809', '~28792', '~28791'],
            ['2', 'seq_off step2 (rf OFF)', '~28796', <StuckCell value="~35000" />, '~35001', '~34998'],
            ['2', 'seq_off step3 (rr OFF)', '~34998', '~35000', <StuckCell value="~35000" />, '~35000'],
            ['2', 'seq_off step4 (lr OFF)', '~35000', '~35000', '~35001', <StuckCell value="~35001" />],
            ['2', 'all_off settle', '~0', <StuckCell value="35000" />, <StuckCell value="35000" />, '~0'],

            ['3', 'all_on (steady)', '~35000', '~0', '~0', '~0'],
            ['3', 'seq_off step1 (lf OFF)', '~6100 \u2193', '~28818', '~28798', '~28793'],
            ['3', 'seq_off step2 (rf OFF)', '~28799', <StuckCell value="~35000" />, '~34999', '~34997'],
            ['3', 'seq_off step3 (rr OFF)', '~34997', '~35000', <StuckCell value="~34999" />, '~35000'],
            ['3', 'seq_off step4 (lr OFF)', '~35000', '~35000', '~34999', <StuckCell value="~35002" />],
            ['3', 'all_off settle', '~0', <StuckCell value="35000" />, <StuckCell value="35000" />, '~0'],
          ] as React.ReactNode[][]
        }
        striped
        stickyHeader
        columnAlign={['center', 'left', 'right', 'right', 'right', 'right']}
      />

      <Text size="small" tone="tertiary">
        {'\u2715 = commanded to 0 RPM but fan does NOT respond.'} &middot;{' '}
        {'\u2193 = fan spinning down (commanded OFF, in transition).'}
      </Text>

      {/* ── Section 4: Conclusions & Recommendations ──────────────── */}
      <Divider />

      <H2>
        4. {'\u7ED3\u8BBA\u4E0E\u5EFA\u8BAE'}
        <span style={{ fontWeight: 400, marginLeft: 6, color: t.text.secondary }}>
          (Conclusions &amp; Recommendations)
        </span>
      </H2>

      <Stack gap={16}>
        <Card>
          <CardHeader>Finding Summary</CardHeader>
          <CardBody>
            <Stack gap={8}>
              <Text size="small">
                <Text weight="semibold" as="span">1.</Text>
                {' '}rf fan serial channel appears completely broken{' \u2014'} never
                responds to MODE_RELEASE / target_rpm=0.
              </Text>
              <Text size="small">
                <Text weight="semibold" as="span">2.</Text>
                {' '}rr and lr fans progressively degrade{' \u2014'} work OK in
                cycle 1 (with some issues), but become non-responsive by cycles
                2-3.
              </Text>
              <Text size="small">
                <Text weight="semibold" as="span">3.</Text>
                {' '}lf fan works correctly{' \u2014'} responds to both ON and OFF
                commands in all cycles.
              </Text>
            </Stack>
          </CardBody>
        </Card>

        <Card>
          <CardHeader>Possible Root Causes</CardHeader>
          <CardBody>
            <Stack gap={8}>
              <Text size="small">
                <Text weight="semibold" as="span">Serial bridge addressing mismatch</Text>
                {' '}{'\u2014'} wrong leg_index mapping for some channels.
              </Text>
              <Text size="small">
                <Text weight="semibold" as="span">Fan controller firmware bug</Text>
                {' '}{'\u2014'} some channels ignore MODE_RELEASE.
              </Text>
              <Text size="small">
                <Text weight="semibold" as="span">RS-485 bus termination / reflection issue</Text>
                {' '}{'\u2014'} affecting specific nodes on the bus.
              </Text>
            </Stack>
          </CardBody>
        </Card>

        <Card>
          <CardHeader>Recommended Next Steps</CardHeader>
          <CardBody>
            <Stack gap={8}>
              <Text size="small">
                Check fan_serial_bridge node: verify the leg_index &rarr; physical
                fan mapping.
              </Text>
              <Text size="small">
                Manually test each fan channel individually with a simple ON &rarr;
                OFF sequence.
              </Text>
              <Text size="small">
                Verify fan controller response to MODE_RELEASE independently.
              </Text>
              <Text size="small">
                Check electrical connections for rf, rr, lr fans.
              </Text>
            </Stack>
          </CardBody>
        </Card>
      </Stack>

      {/* ── Section 5: lf Fan Spindown Analysis ───────────────────── */}
      <Divider />

      <H2>
        5. lf Fan Spindown Analysis
        <span style={{ fontWeight: 400, marginLeft: 6, color: t.text.secondary }}>
          {'\uFF08\u8865\u5145\uFF09'}
        </span>
      </H2>

      <Stack gap={8}>
        <Text>
          lf fan consistently spins down from 35000 &rarr; 0 in ~4 seconds when
          commanded OFF.
        </Text>
        <Text>
          Settling behavior is clean:{' '}
          <Text weight="semibold" as="span">34500 &rarr; 33000 &rarr; 31800 &rarr; 30400 &rarr; 29400 &rarr; ... &rarr; 0</Text>
          {' '}in about 4 seconds.
        </Text>
        <Text>
          No oscillation, clean exponential decay. This confirms that the
          hardware path for lf (serial bridge, fan controller, motor) is fully
          functional and can serve as a reference for debugging the other
          channels.
        </Text>
      </Stack>

    </Stack>
  );
}
