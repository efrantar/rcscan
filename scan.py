from subprocess import Popen, PIPE


class Scanner:

    def __init__(self, rectfile='', scantbl='', fix_centers=False, cwd='.'):
        self.rectfile = rectfile
        self.scantbl = scantbl
        self.fix_centers = fix_centers
        self.cwd = cwd

    def connect(self):
        cmd = ['./scan']
        if self.rectfile:
            cmd += ['-r', self.rectfile]
        if self.scantbl:
            cmd += ['-t', self.scantbl]
        if self.fix_centers:
            cmd += ['-f']
        self.proc = Popen(cmd, stdin=PIPE, stdout=PIPE, cwd=self.cwd)
        while 'Ready!' not in self.proc.stdout.readline().decode():
            pass # wait for everything to boot up
        return self

    def disconnect(self):
        self.proc.terminate()

    def __enter__(self):
        return self.connect()

    def __exit__(self, exception_type, exceptioN_value, traceback):
        self.disconnect()

    def load(self, path):
        self.proc.stdin.write(('load %s\n' % path).encode())
        self.proc.stdin.flush() # send command instantly
        self.proc.stdout.readline() # clear "Ready!"

    def scan(self, remap=True):
        self.proc.stdin.write('scan\n'.encode())
        self.proc.stdin.flush()
        facecube = self.proc.stdout.readline().decode()[:-1] # strip '\n'
        self.proc.stdout.readline() # clear time 
        self.proc.stdout.readline()
        if 'Error' in facecube:
            return ''
        facecube = list(facecube)
        if remap:
            for i, c in enumerate('URFDLB'):
                facecube[9 * i + 4] = c
        else:
            cmap = {c: facecube[9 * i + 4] for i, c in enumerate('URFDLB')}
            for i in range(len(facecube)):
                if i % 9 != 4:
                    facecube[i] = cmap[facecube[i]] 
        return ''.join(facecube)


if __name__ == '__main__':
    import os 

    with Scanner() as scanner:
        print('Scanner ready.')

        count = 0
        succs = 0
        for dir in os.listdir('data'):
            for file in os.listdir(os.path.join('data', dir)):
                if file == 'scan.rects':
                    continue
                scanner.load(os.path.join('data', dir, file))
                facecube = file.split('.')[0] 
                scanned = scanner.scan()
                if scanned == facecube:
                    succs += 1
                    print('OK.')
                else:
                    print('Error.' if scanned == '' else 'Mismatch.', os.path.join(dir, file))
                count += 1 
        print('Correctly scanned: %d/%d' % (succs, count))

