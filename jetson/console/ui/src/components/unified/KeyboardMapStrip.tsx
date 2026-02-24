function Key({ label, wide = false }: { label: string; wide?: boolean }): JSX.Element {
  return <div className={`unified-kb-key ${wide ? "wide" : ""}`}>{label}</div>;
}

export function KeyboardMapStrip(): JSX.Element {
  return (
    <div className="unified-keyboard-strip">
      <div className="unified-keyboard-title">Keyboard Map: Base = arrows + SPACE / SHIFT / CTRL / X - Arm 1-6 + [ ] O P H N B</div>
      <div className="unified-keyboard-layout">
        <Key label="1" />
        <Key label="<" />
        <Key label=">" />
        <Key label="..." />
        <Key label="1" />
        <Key label="<-" />
        <Key label="->" />
        <Key label="[" />
        <Key label="]" />
        <Key label="SPACE" wide />
        <Key label="1" />
        <Key label="2" />
        <Key label="3" />
        <Key label="4" />
      </div>
      <div className="unified-keyboard-legend">
        <strong>Base:</strong> ARROWS + SPACE / SHIFT / CTRL / X - <strong>Arm:</strong> 1 2 3 4 5 6 = J1 J2 J3 J4 J5 J0 [O]
      </div>
    </div>
  );
}
