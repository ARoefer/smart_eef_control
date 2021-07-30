const express = require('express');
const app = express();

var port = 8000;

if (process.argv.length > 2) {
  port = Number(process.argv[2]);
}

app.get('/', (req, res) => {
    res.sendFile(`${__dirname}/static/cart_controller.html`)
});

app.use('/static', express.static('static'));
app.use('/nipplejs', express.static('node_modules/nipplejs/dist/'));

app.listen(port, () => {
  console.log(`Express app listening at http://localhost:${port}`)
});