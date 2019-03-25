let opCommand = "mov";
let processArgs = process.argv;
let charComment = false;
let stringTerminatorChar = true;
let argCount = 0;

for (let i = 2; i < processArgs.length; i++) {
  if (processArgs[i] === "-op") {
    if (processArgs[i + 1] !== undefined) {
      opCommand = processArgs[i + 1];
      argCount += 2;
    }
  } else if (processArgs[i] === "-c") {
    charComment = true;
    argCount++;
  } else if (processArgs[i] === "-nt") {
    stringTerminatorChar = false;
    argCount++;
  }
}

if (processArgs.length < argCount + 3) {
  console.error("Not enough arguments.");
  process.exit(1);
}

function convertToHex(inputChar, charIndex) {
  if (inputChar === 0x00) {
    if (charComment) {
      return `0x00 // Input (${charIndex}): \\0`;
    }
    return "0x00";
  }

  if (charComment) {
    return `0x${inputChar.charCodeAt(0).toString(16)} // Input (${charIndex}): ${inputChar[0]}`;
  }
  return "0x" + inputChar.charCodeAt(0).toString(16);
}

function pad(str, size, withChar = "0") {
  var s = str + "";
  while (s.length < size) s = withChar + s;
  return s;
}

for (let i = argCount + 2; i < processArgs.length; i += 2) {
  if (processArgs[i] === "-op") {
    continue;
  }

  if (processArgs[i] === "-c") {
    i--;
    continue;
  }
  if (processArgs[i + 1] === undefined) {
    console.error("No string arguement at param", i);
    process.exit(2);
  }
  
  let startingAddress = 0;
  try {
    startingAddress = parseInt(processArgs[i]);
  } catch (err) {
    console.error(err);
  }

  let processString = processArgs[i + 1];

  for (let j = 0; j < processString.length; j++) {
    console.log(`${opCommand} 0x${pad((startingAddress + j).toString(16), 4)} ${convertToHex(processString[j], j + 1)}`);
  }
  if (stringTerminatorChar) {
    console.log(`${opCommand} 0x${pad((processString.length).toString(16), 4)} ${convertToHex(0x00, processString.length)}`);
  }
  console.log("");
}

