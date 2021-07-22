const express = require('express')
const app = express()
const port = 8000

app.get('/', (req, res) => {
    res.sendFile(`${__dirname}/static/cart_controller.html`)
});

app.use('/static', express.static('static'));
app.use('/nipplejs', express.static('node_modules/nipplejs/dist/'));

app.listen(port, () => {
  console.log(`Express app listening at http://localhost:${port}`)
});