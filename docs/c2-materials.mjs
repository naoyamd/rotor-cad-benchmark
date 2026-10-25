// Display finishes inferred from the C2 BOM; not a manufacturing specification.
export function surfaceFor(part) {
  const material=part.material || '';
  if (/Elastomer/i.test(material)) return {color:0x202123,metalness:0,roughness:.92};
  if (/Composite/i.test(material)) return {color:0x30343a,metalness:.08,roughness:.72};
  if (/Polymer/i.test(material)) return {color:0x34363a,metalness:0,roughness:.8};
  if (/Bronze/i.test(material)) return {color:0x927344,metalness:.65,roughness:.42};
  if (/Rod|Piston|Pin|Ball|Race|SliderSphere|Mast$/.test(part.name)) return {color:0xb6bac0,metalness:.7,roughness:.28};
  if (/Housing|Carrier|Cradle|Fork|Guide|Body|Stator|RotorRing/.test(part.name)) return {color:0x737b84,metalness:.42,roughness:.62};
  return {color:0x969da5,metalness:.6,roughness:.42};
}
