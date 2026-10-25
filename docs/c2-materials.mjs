// BOM materials plus illustrative coatings; not an aircraft finish specification.
export function surfaceFor(part) {
  const material=part.material || '';
  if (/Elastomer/i.test(material)) return {color:0x202123,metalness:0,roughness:.92};
  if (/Composite/i.test(material)) return {color:0x30343a,metalness:.08,roughness:.72};
  if (/Polymer/i.test(material)) return {color:0x34363a,metalness:0,roughness:.8};
  if (/Bronze/i.test(material)) return {color:0x927344,metalness:.65,roughness:.42};
  if (/FlightLockHead|MastIndexPin$/.test(part.name)) return {color:0x9c483c,metalness:.12,roughness:.58};
  if (/Retainer|PinHead|RaceBolts|SocketSplitBolts/.test(part.name)) return {color:0xb59b66,metalness:.58,roughness:.38};
  if (/LockGuide|LockWeb/.test(part.name)) return {color:0x9a8c58,metalness:.18,roughness:.65};
  if (/StatorSocket|RotorRing|FixedGuide|GuideBridge|CarrierSpoke/.test(part.name)) return {color:0x7d8c6b,metalness:.12,roughness:.65};
  if (/Rod|Piston|Pin|Ball|Race|SliderSphere|Mast$/.test(part.name)) return {color:0xb6bac0,metalness:.7,roughness:.28};
  if (/Housing|Carrier|Cradle|Fork|Guide|Body/.test(part.name)) return {color:0x526775,metalness:.16,roughness:.62};
  return {color:0x969da5,metalness:.6,roughness:.42};
}
