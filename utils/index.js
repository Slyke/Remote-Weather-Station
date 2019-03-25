const express = require('express');
bodyParser = require('body-parser');
const app = express();

let argPortIndex = process.argv.indexOf("-p") > 0 ? process.argv[process.argv.indexOf("-p") + 1] : undefined;

const port = argPortIndex || 3000;

app.use(bodyParser.json());

var lastOutput = {};

app.get('*', getLastOutput);
app.post('*', consoleLogOutput);

function consoleLogOutput(req, res) {
  var ip = req.headers['x-forwarded-for'] || req.connection.remoteAddress;
  lastOutput = req.body;
  console.log(Math.round(new Date().getTime() / 1000).toString());
  console.log("Received from: ", ip);
  console.log(req.body);
  console.log("");
  res.status(200).send({status: "ok"});
}

function getLastOutput(req, res) {
  res.status(200).send(lastOutput)
}

app.listen(port, '0.0.0.0', () => console.log(`listening on port ${port}!`));
