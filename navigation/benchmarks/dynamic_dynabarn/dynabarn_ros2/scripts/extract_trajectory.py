#!/usr/bin/env python3
"""Read unrolled DynaBARN keyframes; never load/execute a Classic binary.
ELF support is fail-closed for the distributed x86-64 O0 GCC plugin pattern.
Prefer original .cc sources when available.
"""
import re,struct,subprocess,hashlib,math
from pathlib import Path

def extract(path):
    path=Path(path)
    if path.suffix in ('.cc','.cpp'):
        text=path.read_text()
        duration=float(re.search(r'new gazebo::common::PoseAnimation\([^,]+,\s*([\d.]+)',text)[1])
        frames=[[float(x) for x in row] for row in re.findall(r'CreateKeyFrame\(([-\d.]+)\);\s*key->Translation\(ignition::math::Vector3d\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\)',text)]
        method='original_cpp_keyframes'
    else:
        data=path.read_bytes()
        if data[:6]!=b'\x7fELF\x02\x01':raise ValueError('Requires little-endian ELF64')
        header=struct.unpack_from('<16sHHIQQQIHHHHHH',data)
        if header[2]!=62:raise ValueError('Only x86-64 plugin extraction is supported')
        segments=[]
        for i in range(header[10]):
            h=struct.unpack_from('<IIQQQQQQ',data,header[5]+i*header[9])
            if h[0]==1:segments.append((h[3],h[2],h[5]))
        def constant(address):
            for v,o,n in segments:
                if v<=address and address+8<=v+n:return struct.unpack_from('<Q',data,o+address-v)[0]
            raise ValueError(f'Unmapped constant {address:x}')
        def number(bits):
            if not isinstance(bits,int):raise ValueError('Unresolved floating-point argument')
            return struct.unpack('<d',struct.pack('<Q',bits))[0]
        text=subprocess.check_output(['objdump','-d','-C','--no-show-raw-insn',str(path)],text=True)
        match=re.search(r'^0+[0-9a-f]+ <gazebo::[^\n]*::Load\([^\n]+\)>:\n(.*?)(?=^0+[0-9a-f]+ <|\Z)',text,re.M|re.S)
        if not match:raise ValueError('No unrolled Gazebo model Load() found')
        regs={};memory={};objects={};frames=[];rotations=[];duration=None
        def value(token,line):
            token=token.strip()
            if '%rip' in token:
                return constant(int(re.search(r'#\s*([0-9a-f]+)',line)[1],16))
            if token.startswith('%'):return regs.get(token)
            if token.startswith('$'):return int(token[1:],0)
            if token.endswith('(%rbp)'):return memory.get(token)
            return None
        def store(token,val):
            if token.startswith('%'):regs[token]=val
            elif token.endswith('(%rbp)'):memory[token]=val
        for line in match[1].splitlines():
            m=re.match(r'\s*[0-9a-f]+:\s+(\w+)\s*(.*)',line)
            if not m:continue
            op,arg=m.groups();arg=arg.split('#')[0].strip()
            if op.startswith('mov') and ',' in arg:
                a,b=arg.split(',',1);store(b.strip(),value(a,line))
            elif op=='lea' and ',' in arg:
                a,b=arg.split(',',1);store(b.strip(),('stack',a.strip()) if a.strip().endswith('(%rbp)') else None)
            elif op in ('pxor','xorpd','xor') and ',' in arg:
                a,b=arg.split(',',1)
                if a.strip()==b.strip():store(b.strip(),0)
            elif op.startswith('call'):
                if 'PoseAnimation::PoseAnimation(' in line:duration=number(regs.get('%xmm0'))
                if 'PoseAnimation::CreateKeyFrame(' in line:frames.append([number(regs.get('%xmm0')),None,None,None])
                if 'Vector3<double>::Vector3(double const&' in line or 'Quaternion<double>::Quaternion(double const&' in line:
                    args=[]
                    for name in ('%rsi','%rdx','%rcx'):
                        ref=regs.get(name)
                        if not isinstance(ref,tuple):raise ValueError('Unsupported constructor argument')
                        args.append(number(memory.get(ref[1])))
                    objects[regs.get('%rdi')]=args
                if 'PoseKeyFrame::Translation(' in line:
                    if not frames:raise ValueError('Translation without keyframe')
                    frames[-1][1:]=objects[regs.get('%rsi')]
                if 'PoseKeyFrame::Rotation(' in line:rotations.append(objects[regs.get('%rsi')])
                for reg in list(regs):
                    if reg in ('%rax','%rcx','%rdx','%rsi','%rdi','%r8','%r9','%r10','%r11') or reg.startswith('%xmm'):regs[reg]=None
        if len(rotations)!=len(frames) or any(any(abs(v)>1e-10 for v in r) for r in rotations):raise ValueError('Only zero-rotation DynaBARN animations supported')
        method='elf64_constant_dataflow_no_execution'
    if not duration or len(frames)<2 or any(v is None or not math.isfinite(v) for f in frames for v in f):raise ValueError('Incomplete trajectory')
    if abs(frames[0][0])>1e-9 or abs(frames[-1][0]-duration)>1e-6 or any(b[0]<=a[0] for a,b in zip(frames,frames[1:])):raise ValueError('Invalid keyframe timing')
    return {'duration':duration,'keyframes':frames,'extraction':method,'source':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}

if __name__=='__main__':
    import sys,json
    print(json.dumps(extract(sys.argv[1]),indent=2))
